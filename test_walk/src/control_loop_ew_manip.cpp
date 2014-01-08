#include <ros/ros.h>
#include <ros/package.h>
#include <ros/subscribe_options.h>
#include <boost/thread.hpp>
#include <boost/algorithm/string.hpp>
#include <stdio.h>

#include <test_walk/control_loop_ew_manip.h>
#include <test_walk/cmu_ctrl_utils.h>

#include <cmu_walk/TrajEW.h>
#include <cmu_walk/RobotState.h>

extern std::string pkg_name;

////////////////////////////////////////////////////////////
static void pre_enable_ew_manip(const atlas_msgs::AtlasState &data_from_robot, 
    boost::mutex &data_from_robot_lock);

static std::map <const std::string, double *> drillBalanceLookup;

//cache for text messages
static atlas_ros_msgs::simple_text *txt;

static char realMessageBuf[2000];

static unsigned char txt_seq = 0;
void sendMessageD(const char *msg) {
	strcat(realMessageBuf, msg);
	txt->text = std::string(realMessageBuf);
	txt->seq = txt_seq++;
	fprintf(stderr, "%s",msg);
}

static char messageBuf[1000];

void sendMessageD() {
	sendMessageD(messageBuf);
}


bool MBY_HARD_STOP = false;

static double ntInc = 0.01;
static double nrInc = 0.02;


// sfeng states
static CMUCtrlUtils cmu_utils;

static PelvRobotState RS;
//static KinematicFilter3 kcekf;
static KinematicFilterEric kcekf;
static PelvIkCon IK;
static PelvIdCon ID;
static IKcmd IK_d;
static IDcmd ID_d;
static BatchLogger logger; 

static double initYaw;
static double tc;	//time since controller started
static double tc0; //time controller started
static double ts0; //time segment started
static double ts;  //time since segment started
static int step = 0;

static double relaxThresh = 0.03;
static double relaxGain = 0.001;

static bool finishSeg = false;
static int conType;

static const Eigen::Vector3d angGainStiff(1,1,1);
static const Eigen::Vector3d angGainNoAxial(0,1,1);
static const Eigen::Vector3d angGainNone(0,0,0);

static bool newCommand = false;
static double commandData[100];
static int commandInfo[100];
static int NcmdDat;
static int NcmdInf;

static TrajEW handTraj[2][3];
static TrajEW jointTraj[28];
static TrajEW CoMtraj[3];
static TrajEW angleTraj;
static Eigen::Vector3d circleCenter;
static Eigen::Vector3d circleAxis;
static Eigen::Vector3d circleRadius;
int circleSide = 0;
static double duration;
static Eigen::Quaterniond handQ0[2], handQf[2];
static Eigen::Quaterniond pelvQ0, pelvQf, torsoQ0, torsoQf;
static double handdd[2][3], comdd[3];	//for passing to the ID
static bool conHand[2];
static int trajMode = 0;
static double desNeck;

static double home[3] = {0.045, 0, 0.9};
static double homeGain[3] = {0.0003, 0.0003, 0};

static double comIntX = 0.0;
static double comIntTimeConst = 10.0;
static double footIntTimeConst = 1.0;
static double comIntLim = 0.04;

const static Eigen::Quaterniond quatIdent = EA2quat(0,0,0);
const static Eigen::Quaterniond pelvisBack = EA2quat(0, -0.48, 0);

static Eigen::Quaterniond homeTorso = quatIdent;
static Eigen::Quaterniond homePelvis = quatIdent;
static double homeTorsoGain = 0.0003;

static int keyboardCommand = -1;
static bool keyboardActive[2] = {false, false};

static double MBYlockTime = 5.0;
static double MBYunlockTime = 5.0;
const static double MBYlockAng = 0.48;

bool jointsByTraj[28] = {false};

void noneByTraj() 	{  for(int i = 0; i < 28; i++)		jointsByTraj[i] = false;  }
void allByTraj() 	{  for(int i = 0; i < 28; i++)		jointsByTraj[i] = true;  }
void armByTraj() 	{  for(int i = 16; i < 28; i++)		jointsByTraj[i] = true;   }
void armByTraj(int side) 	{  for(int i = 0; i < 6; i++)		jointsByTraj[16+i+6*side] = true;   }
void spineByTraj() 	{  for(int i = 0; i < 4; i++)		jointsByTraj[i] = true;   }

bool IKDclearedAlready=false;

void clearIKD() {
	fprintf(stderr, "want to clearIK()\n");
	if(!IKDclearedAlready) {
		fprintf(stderr, "clearIK()\n");
		IKDclearedAlready = true;
		IK_d.setToRS((PelvRobotState &)*IK.ikrs);
		IK_d.setVel0();
	}
}

static const double demoPose[28] = {
		0,0,0,0,
		0,0,0,0,0,0,
		0,0,0,0,0,0,
		-0.45, -0.7, 1.97, 1.67, 0.6, 0.89,
		-0.92, 0.46, 1.94, -1.27, 0.29, -0.87
		//0,0,0,0,0,0,
		//0,0,0,0,0,0
};

void demoArmScript() {
	duration = 5.0;
	for(int i = 0; i < 28; i++)		jointTraj[i].freshMove(IK.ikrs->joints[i], demoPose[i], duration);
	noneByTraj();
	armByTraj();
	IK.hand_weight_p = 0.0;
	//ID.handWeightR = ID.handWeightP = 0.0;
	clearIKD();
	IK_d.rotGain[LEFT] = IK_d.rotGain[RIGHT] = &angGainNone;
}

bool initCoMshift() {
	duration = commandData[3];
	if(duration <= 0) {
		sendMessageD("Duration must be greater than 0\n");
		return false;
	}

	IK.restoreDefaults();
	ID.restoreDefaults();
	noneByTraj();


	//IK_d.rotGain[LEFT] = IK_d.rotGain[RIGHT] = &angGainStiff;



	if(NcmdInf >= 2) {
		if(commandInfo[1] == 1) {	//if hands move with body
			armByTraj();
			for(int i = 0; i < 28; i++)		jointTraj[i].setConstant(IK.ikrs->joints[i]);
			IK_d.skipHand[LEFT] = IK_d.skipHand[RIGHT] = true;
			//ID_d.skipHand[LEFT] = ID_d.skipHand[RIGHT] = true;
		}
	}
	else {	//stationary hands
		IK_d.skipHand[LEFT] = IK_d.skipHand[RIGHT] = false;
		//ID_d.skipHand[LEFT] = ID_d.skipHand[RIGHT] = false;
		for(int s = 0; s < 2; s++) {
			IK_d.handQ[s] = IK.ikrs->handQ[s];
			for(int i = 0; i < 3; i++)	IK_d.hand[s][i] = IK.ikrs->hand[s][i];
		}
	}



	CoMtraj[XX].freshMove(IK.ikrs->com[XX], IK.ikrs->com[XX]+commandData[XX], duration, Linear);
	CoMtraj[YY].freshMove(IK.ikrs->com[YY], IK.ikrs->com[YY]+commandData[YY], duration, Linear);
	CoMtraj[ZZ].freshMove(IK.ikrs->root[ZZ], IK.ikrs->root[ZZ]+commandData[ZZ], duration, Linear);

	for(int i = 0; i < 3; i++) {
		home[i] += commandData[i];
	}


	double dEA[3] = {0,0,0};
	if(NcmdDat >= 5)	dEA[0] = commandData[4];
	if(NcmdDat >= 6)	dEA[1] = commandData[5];
	if(NcmdDat >= 7)	dEA[2] = commandData[6];

	Eigen::Quaterniond bodyRotate = EA2quat(dEA[0], dEA[1], dEA[2]);

	pelvQ0 = IK_d.pelvQ;
	pelvQf = bodyRotate*pelvQ0;

	torsoQ0 = IK_d.torsoQ;
	torsoQf = bodyRotate*torsoQ0;

	homeTorso = bodyRotate*homeTorso;
	homePelvis = bodyRotate*homePelvis;		//probably reset constantly


	sprintf(messageBuf, "CoM shift of {%g, %g, %g} and rotation of {%g, %g, %g} in %g seconds\n",commandData[XX], commandData[YY], commandData[ZZ], dEA[0], dEA[1], dEA[2], duration);
	sendMessageD();

	return true;
}

static const double poses[1][12] = {
		{0.52, -1.52, 2.28, 1.49, 0, 1.15, 0.52, 1.52, 2.28, -1.49, 0, -1.15}
};

bool initPreDefinedArms(int pose) {

	if(pose > 0)	return false;
	sprintf(messageBuf, "Going to pose %d\n",pose);
	sendMessageD();


	switch(pose) {
	default:
		duration = 5.0;
		break;
	}

	IK.restoreDefaults();
	ID.restoreDefaults();

	noneByTraj();
	armByTraj();

	clearIKD();

	IK_d.skipHand[LEFT] = true;
	//ID_d.skipHand[LEFT] = true;
	IK_d.skipHand[RIGHT] = true;
	//ID_d.skipHand[RIGHT] = true;
	IK_d.relHands = false;

	IK_d.rotGain[LEFT] = &angGainNone;
	IK_d.rotGain[RIGHT] = &angGainNone;



	//keeps track of most recent hand moved

	keyboardActive[LEFT] = true;
	keyboardActive[RIGHT] = true;

	for(int i = 0; i < 28; i++)		jointTraj[i].setConstant(IK.ikrs->joints[i]);

	for(int i = 0; i < 12; i++) 		jointTraj[16+i].freshMove(IK.ikrs->joints[16+i], poses[pose][i], duration);

	return true;
}

int djSide;

bool initDirectJoints() {
	if(NcmdDat >= 7) 	duration = commandData[6];
	else {
		duration = 2.0;
		sendMessageD("Using the default time of 2.0 seconds.  Please specify a time.\n");
	}

	if(duration <= 0) {
		sendMessageD("Time must be greater than 0.\n");
		return false;
	}



	int side = commandInfo[1];
	djSide = side;
	IK.restoreDefaults();
	ID.restoreDefaults();

	noneByTraj();
	armByTraj(side);

	clearIKD();

	IK_d.skipHand[side] = true;
	//ID_d.skipHand[side] = true;
	IK_d.relHands = false;

	if(NcmdInf >= 3 && commandInfo[2] == 1) {	//other hand locks joints
		IK_d.skipHand[1-side] = true;
		armByTraj(1-side);
	}
	else {	//other hand maintains position
		if(IK_d.skipHand[1-side]) {
			IK_d.handQ[1-side] = IK.ikrs->handQ[1-side];
			for(int i = 0; i < 3; i++) IK_d.hand[1-side][i] = IK.ikrs->hand[1-side][i];
		}
		IK_d.skipHand[1-side] = false;
		IK_d.rotGain[1-side] = &angGainStiff;
	}


	//ID_d.skipHand[1-side] = false;


	IK_d.rotGain[side] = &angGainNone;



	//keeps track of most recent hand moved
	keyboardActive[side] = true;
	keyboardActive[1-side] = false;


	for(int i = 0; i < 28; i++)		jointTraj[i].setConstant(IK.ikrs->joints[i]);

	for(int i = 0; i < 6; i++) 		jointTraj[16+6*side+i].freshMove(IK.ikrs->joints[16+6*side+i], commandData[i], duration);

	return true;
}

bool initMBYlock() {
	if(MBY_HARD_STOP) {
		sendMessageD("Already locked.\n");
		return false;
	}

	duration = MBYlockTime;

	IK.restoreDefaults();
	ID.restoreDefaults();

	noneByTraj();
	spineByTraj();
	armByTraj();

	clearIKD();

	for(int i = 0; i < 28; i++)		jointTraj[i].setConstant(IK.ikrs->joints[i]);
	jointTraj[1].freshMove(IK.ikrs->joints[1], MBYlockAng, duration, Linear);

	IK_d.skipHand[LEFT] = IK_d.skipHand[RIGHT] = true;
	IK_d.relHands = false;
	torsoQf = torsoQ0 = IK_d.torsoQ;

	pelvQ0 = IK_d.pelvQ;
	pelvQf = pelvisBack*torsoQf;

	return true;
}

bool initMBYunlock() {
	if(!MBY_HARD_STOP) {
		sendMessageD("Already unlocked.\n");
		return false;
	}
	double torsoEA[3];
	quat2EA(IK.ikrs->utorsoq, torsoEA);
	if(torsoEA[1] > IK.torsoPitchLimit[1]) {
		sendMessageD("Unsafe to unlock\n");
		return false;
	}
	MBY_HARD_STOP = false;

	duration = MBYunlockTime;

	IK.restoreDefaults();
	ID.restoreDefaults();

	noneByTraj();
	armByTraj();

	clearIKD();

	for(int i = 0; i < 28; i++)		jointTraj[i].setConstant(IK.ikrs->joints[i]);

	IK_d.skipHand[LEFT] = IK_d.skipHand[RIGHT] = true;
	IK_d.relHands = false;

	torsoQf = torsoQ0 = IK_d.torsoQ;

	pelvQ0 = IK_d.pelvQ;
	pelvQf = torsoQf;

	return true;
}

void initLockArms() {
	sendMessageD("begin lock arms mode\n");
	keyboardCommand = -1;	//clear any lingering commands
	for(int i = 0; i < 28; i++)		jointTraj[i].setConstant(IK.ikrs->joints[i]);
	noneByTraj();

	IK.restoreDefaults();
	ID.restoreDefaults();
	ID_d.skipHand[LEFT] = ID_d.skipHand[RIGHT] = false;
	for(int s = 0; s < 2; s++) 	if(IK_d.skipHand[s]) 		armByTraj(s);

}

bool initToTargetFromCommand() {
	//printing
	sendMessageD("New motion command (or dummy command for mode switching)\n");

	//traj mode
	trajMode = commandInfo[0];

	switch(trajMode) {
	case 0:
		if(NcmdDat < 4) {
			sprintf(messageBuf, "Need more parameters (%d of 4 given)\n", NcmdDat);
			sendMessageD();
			return false;
		}
		duration = commandData[3];
		if(duration <= 0) {
			sendMessageD("This command requires a positive duration\n");
			return false;
		}
		break;
	case 1:
		if(NcmdDat < 5) {
			sprintf(messageBuf, "Need more parameters (%d of 5 given)\n", NcmdDat);
			sendMessageD();
			return false;
		}
		duration = commandData[3];
		if(duration <= 0) {
			sendMessageD("This command requires a positive duration\n");
			return false;
		}

		if(NcmdInf < 2) {
			sendMessageD("Need to specify a hand\n");
			return false;
		}
		circleSide = commandInfo[1];
		if(circleSide < 0 || circleSide > 1) {
			sprintf(messageBuf, "Bad hand %d\n", circleSide);
			sendMessageD();
			return false;
		}
		break;
	case 3:
		if(NcmdDat < 2) {
			sprintf(messageBuf, "Need more parameters (%d of 2 given)\n", NcmdDat);
			sendMessageD();
			return false;
		}
		duration = commandData[1];
		if(duration <= 0) {
			sendMessageD("This command requires a positive duration\n");
			return false;
		}
		if(NcmdInf < 2 || (commandInfo[1] != 0 && commandInfo[1] != 1)) {
			sendMessageD("You must specify a hand to plunge\n");
			return false;
		}
		break;
	default:
		sendMessageD("Unknown command Type\n");
		return false;
		break;
	}



	IK.restoreDefaults();
	ID.restoreDefaults();

	//will clear one if only one hand
	conHand[LEFT] = conHand[RIGHT] = true;
	IK_d.skipHand[LEFT] = IK_d.skipHand[RIGHT] = false;
	noneByTraj();
	keyboardActive[LEFT] = keyboardActive[RIGHT] = true;

	//if we should lock the joints on the uncommanded arm
	//other choice is maintain current position
	bool lockOtherArm = false;
	if(NcmdInf >= 4 && commandInfo[3] == 1)	lockOtherArm = true;


	int whichHand = -1;
	if(NcmdInf >= 2)		whichHand = commandInfo[1];
	//which hand
	switch(whichHand) {
	case LEFT:
		handQf[RIGHT] = handQ0[RIGHT];
		if(lockOtherArm) {
			conHand[RIGHT] = false;
			IK_d.skipHand[RIGHT] = true;
			armByTraj(RIGHT);
			for(int i = 22; i < 28; i++) 		jointTraj[i].setConstant(IK.ikrs->joints[i]);
		}
		else {
			for(int i = 0; i < 3; i++)			handTraj[RIGHT][i].setConstant(IK.ikrs->hand[RIGHT][i]);
		}
		keyboardActive[RIGHT] = false;
		if(IK_d.relHands) clearIKD();
		IK_d.relHands = false;
		break;
	case RIGHT:
		handQf[LEFT] = handQ0[LEFT];
		if(lockOtherArm) {
			conHand[LEFT] = false;
			IK_d.skipHand[LEFT] = true;
			armByTraj(LEFT);
			for(int i = 16; i < 22; i++) 		jointTraj[i].setConstant(IK.ikrs->joints[i]);
		}
		else {
			for(int i = 0; i < 3; i++)			handTraj[LEFT][i].setConstant(IK.ikrs->hand[LEFT][i]);
		}
		keyboardActive[LEFT] = false;
		if(IK_d.relHands) clearIKD();
		IK_d.relHands = false;
		break;
	default:
		if(!IK_d.relHands) clearIKD();
		IK_d.relHands = true;
		break;
	}

	int noAxialHand = -1;

	if(NcmdInf >= 3 && trajMode == 0) 	noAxialHand = commandInfo[2];	//translation/rotation
	if(trajMode == 1 || trajMode == 3) {	//circles or plunge/retract
		if(NcmdInf >= 3 && commandInfo[2] == 0) 	noAxialHand = commandInfo[1];	//allow axial rotation
	}

	switch(noAxialHand) {
	case LEFT:
		sendMessageD("setting LEFT axial\n");
		if(IK_d.rotGain[LEFT] != &angGainNoAxial) {
			IK_d.rotGain[LEFT] = &angGainNoAxial;
			clearIKD();
		}
		if(IK_d.rotGain[RIGHT] != &angGainStiff) {
			IK_d.rotGain[RIGHT] = &angGainStiff;
			clearIKD();
		}
		break;
	case RIGHT:
		sendMessageD("setting RIGHT axial\n");
		if(IK_d.rotGain[LEFT] != &angGainStiff) {
			IK_d.rotGain[LEFT] = &angGainStiff;
			clearIKD();
		}
		if(IK_d.rotGain[RIGHT] != &angGainNoAxial) {
			IK_d.rotGain[RIGHT] = &angGainNoAxial;
			clearIKD();
		}
		break;
	case 2:
		sendMessageD("setting both axial\n");
		if(IK_d.rotGain[LEFT] != &angGainNoAxial) {
			IK_d.rotGain[LEFT] = &angGainNoAxial;
			clearIKD();
		}
		if(IK_d.rotGain[RIGHT] != &angGainNoAxial) {
			IK_d.rotGain[RIGHT] = &angGainNoAxial;
			clearIKD();
		}
		break;
	default:
		sendMessageD("setting neither axial\n");
		if(IK_d.rotGain[LEFT] != &angGainStiff) {
			IK_d.rotGain[LEFT] = &angGainStiff;
			clearIKD();
		}
		if(IK_d.rotGain[RIGHT] != &angGainStiff) {
			IK_d.rotGain[RIGHT] = &angGainStiff;
			clearIKD();
		}
		break;
	}



	switch(trajMode) {
	case 1:	//Circles!


		clearIKD();

		angleTraj.freshMove(0, commandData[4], duration, Linear);
		//IK_d.handQ[circleSide] = IK.ikrs->handQ[circleSide];
		for(int i = 0; i < 3; i++) {
			circleCenter[i] = commandData[i]+IK.ikrs->hand[circleSide][i];
			circleAxis[i] = IK.ikrs->handAxis[circleSide][i];
			circleRadius[i] = IK.ikrs->hand[circleSide][i]-circleCenter[i];
		}
		if(NcmdInf >= 4 && commandInfo[3] == 1) {
			circleAxis[XX] = cos(commandData[5]);
			circleAxis[YY] = sin(commandData[5]);
		}
		//remove pitch
		circleAxis[ZZ] = 0.0;
		circleAxis.normalize();
		break;
	case 3:	//plunge/retract

		if(IK_d.relHands) clearIKD();
		else if(IK_d.skipHand[commandInfo[1]]) {
			IK_d.handQ[commandInfo[1]] = IK.ikrs->handQ[commandInfo[1]];
			for(int i = 0; i < 3; i++)	IK_d.hand[commandInfo[1]][i] = IK.ikrs->hand[commandInfo[1]][i];
		}
		for(int s = 0; s < 2; s++) {
			for(int i = 0; i < 3; i++) {
				handTraj[s][i].freshMove(IK_d.hand[s][i], IK_d.hand[s][i]+RS.handAxis[s][i]*commandData[0], duration, Linear);
			}
			handQ0[s] = IK_d.handQ[s];
			handQf[s] = IK.ikrs->handQ[s];
		}
		break;
	default:	//Relative Linear Motion
		double dEA[3] = {0,0,0};
		if(NcmdDat >= 5)	dEA[0] = commandData[4];
		if(NcmdDat >= 6)	dEA[1] = commandData[5];
		if(NcmdDat >= 7)	dEA[2] = commandData[6];
		for(int s = 0; s < 2; s++) {
			if(IK_d.skipHand[s]) {
				IK_d.handQ[s] = IK.ikrs->handQ[s];
				for(int i = 0; i < 3; i++)	IK_d.hand[s][i] = IK.ikrs->hand[s][i];
			}
			handQ0[s] = IK_d.handQ[s];
			Eigen::Quaterniond bodyRotate = EA2quat(dEA[0], dEA[1], dEA[2]);
			handQf[s] = bodyRotate*handQ0[s];
			for(int i = 0; i < 3; i++) {
				handTraj[s][i].freshMove(IK_d.hand[s][i], IK_d.hand[s][i]+commandData[i], duration, Linear);
			}
		}
		break;
	}

	return true;
}



void goToUpperPose() {
	//#ifdef ATLAS_ONLINE
	finishSeg = true;
	//#endif
	if(ts > duration) {
		finishSeg = true;
	}
}

void sendHandMessage(int side) {
	sprintf(messageBuf, "dHand[%d] = {%.3f, %.3f, %.3f}\n",side, IK_d.hand[side][XX]-RS.hand[side][XX],IK_d.hand[side][YY]-RS.hand[side][YY],IK_d.hand[side][ZZ]-RS.hand[side][ZZ]);
	sendMessageD();
}


void lockArms() {
	switch(keyboardCommand) {
	case atlas_ros_msgs::field_param::KEYBOARD_I:
		std::cerr << "in\n";
		for(int s = 0; s < 2; s++) {
			if(keyboardActive[s]) {
				sendHandMessage(s);
				IK_d.hand[s][XX]+=ntInc;
			}
		}
		break;
	case atlas_ros_msgs::field_param::KEYBOARD_O:
		std::cerr << "out\n";
		for(int s = 0; s < 2; s++) {
			if(keyboardActive[s]) {
				sendHandMessage(s);
				IK_d.hand[s][XX]-=ntInc;
			}
		}
		break;
	case atlas_ros_msgs::field_param::KEYBOARD_UP_ARROW:
		std::cerr << "up\n";
		for(int s = 0; s < 2; s++) {
			if(keyboardActive[s]) {
				sendHandMessage(s);
				IK_d.hand[s][ZZ]+=ntInc;
			}
		}
		break;
	case atlas_ros_msgs::field_param::KEYBOARD_DOWN_ARROW:
		std::cerr << "down\n";
		for(int s = 0; s < 2; s++) {
			if(keyboardActive[s]) {
				sendHandMessage(s);
				IK_d.hand[s][ZZ]-=ntInc;
			}
		}
		break;
	case atlas_ros_msgs::field_param::KEYBOARD_LEFT_ARROW:
		std::cerr << "left\n";
		for(int s = 0; s < 2; s++) {
			if(keyboardActive[s]) {
				sendHandMessage(s);
				IK_d.hand[s][YY]+=ntInc;
			}
		}
		break;
	case atlas_ros_msgs::field_param::KEYBOARD_RIGHT_ARROW:
		std::cerr << "right\n";
		for(int s = 0; s < 2; s++) {
			if(keyboardActive[s]) {
				sendHandMessage(s);
				IK_d.hand[s][YY]-=ntInc;
			}
		}
		break;
	case atlas_ros_msgs::field_param::KEYBOARD_S:
		std::cerr << "yaw left\n";
		if(IK_d.relHands) {
			Eigen::Vector3d rel(IK_d.hand[LEFT][XX]-IK_d.hand[RIGHT][XX], IK_d.hand[LEFT][YY]-IK_d.hand[RIGHT][YY], IK_d.hand[LEFT][ZZ]-IK_d.hand[RIGHT][ZZ]);
			Eigen::Quaterniond rot = EA2quat(0, 0, nrInc);
			rel = rot*rel;
			for(int i = 0; i < 3; i++)		IK_d.hand[LEFT][i] = IK_d.hand[RIGHT][i]+rel[i];
			for(int s = 0; s < 2; s++)		IK_d.handQ[s] = rot*IK_d.handQ[s];
		}
		else	for(int s = 0; s < 2; s++)	if(keyboardActive[s])	modifyQ(IK_d.handQ[s], -nrInc,0,0);
		break;
	case atlas_ros_msgs::field_param::KEYBOARD_F:
		std::cerr << "yaw right\n";
		if(IK_d.relHands) {
			Eigen::Vector3d rel(IK_d.hand[LEFT][XX]-IK_d.hand[RIGHT][XX], IK_d.hand[LEFT][YY]-IK_d.hand[RIGHT][YY], IK_d.hand[LEFT][ZZ]-IK_d.hand[RIGHT][ZZ]);
			Eigen::Quaterniond rot = EA2quat(0, 0, -nrInc);
			rel = rot*rel;
			for(int i = 0; i < 3; i++)		IK_d.hand[LEFT][i] = IK_d.hand[RIGHT][i]+rel[i];
			for(int s = 0; s < 2; s++)		IK_d.handQ[s] = rot*IK_d.handQ[s];
		}
		else	for(int s = 0; s < 2; s++)	if(keyboardActive[s])	modifyQ(IK_d.handQ[s], nrInc,0,0);
		break;
	case atlas_ros_msgs::field_param::KEYBOARD_E:
		std::cerr << "pitch up\n";
		if(IK_d.relHands) {
			Eigen::Vector3d rel(IK_d.hand[LEFT][XX]-IK_d.hand[RIGHT][XX], IK_d.hand[LEFT][YY]-IK_d.hand[RIGHT][YY], IK_d.hand[LEFT][ZZ]-IK_d.hand[RIGHT][ZZ]);
			Eigen::Quaterniond rot = EA2quat(0, -nrInc, 0);
			rel = rot*rel;
			for(int i = 0; i < 3; i++)		IK_d.hand[LEFT][i] = IK_d.hand[RIGHT][i]+rel[i];
			for(int s = 0; s < 2; s++)		IK_d.handQ[s] = rot*IK_d.handQ[s];
		}
		else	for(int s = 0; s < 2; s++)	if(keyboardActive[s])	modifyQ(IK_d.handQ[s], 0,0,-nrInc);
		break;
	case atlas_ros_msgs::field_param::KEYBOARD_D:
		std::cerr << "pitch down\n";
		if(IK_d.relHands) {
			Eigen::Vector3d rel(IK_d.hand[LEFT][XX]-IK_d.hand[RIGHT][XX], IK_d.hand[LEFT][YY]-IK_d.hand[RIGHT][YY], IK_d.hand[LEFT][ZZ]-IK_d.hand[RIGHT][ZZ]);
			Eigen::Quaterniond rot = EA2quat(0, nrInc, 0);
			rel = rot*rel;
			for(int i = 0; i < 3; i++)		IK_d.hand[LEFT][i] = IK_d.hand[RIGHT][i]+rel[i];
			for(int s = 0; s < 2; s++)		IK_d.handQ[s] = rot*IK_d.handQ[s];
		}
		else	for(int s = 0; s < 2; s++)	if(keyboardActive[s])	modifyQ(IK_d.handQ[s], 0,0,nrInc);
		break;
	case atlas_ros_msgs::field_param::KEYBOARD_W:
		std::cerr << "roll left\n";
		if(IK_d.relHands) {
			Eigen::Vector3d rel(IK_d.hand[LEFT][XX]-IK_d.hand[RIGHT][XX], IK_d.hand[LEFT][YY]-IK_d.hand[RIGHT][YY], IK_d.hand[LEFT][ZZ]-IK_d.hand[RIGHT][ZZ]);
			Eigen::Quaterniond rot = EA2quat(-nrInc, 0, 0);
			rel = rot*rel;
			for(int i = 0; i < 3; i++)		IK_d.hand[LEFT][i] = IK_d.hand[RIGHT][i]+rel[i];
			for(int s = 0; s < 2; s++)		IK_d.handQ[s] = rot*IK_d.handQ[s];
		}
		else	for(int s = 0; s < 2; s++)	if(keyboardActive[s])	modifyQ(IK_d.handQ[s], 0,nrInc,0);
		break;
	case atlas_ros_msgs::field_param::KEYBOARD_R:
		std::cerr << "roll right\n";
		if(IK_d.relHands) {
			Eigen::Vector3d rel(IK_d.hand[LEFT][XX]-IK_d.hand[RIGHT][XX], IK_d.hand[LEFT][YY]-IK_d.hand[RIGHT][YY], IK_d.hand[LEFT][ZZ]-IK_d.hand[RIGHT][ZZ]);
			Eigen::Quaterniond rot = EA2quat(nrInc, 0, 0);
			rel = rot*rel;
			for(int i = 0; i < 3; i++)		IK_d.hand[LEFT][i] = IK_d.hand[RIGHT][i]+rel[i];
			for(int s = 0; s < 2; s++)		IK_d.handQ[s] = rot*IK_d.handQ[s];
		}
		else	for(int s = 0; s < 2; s++)	if(keyboardActive[s])	modifyQ(IK_d.handQ[s], 0,-nrInc,0);
		break;
	case atlas_ros_msgs::field_param::KEYBOARD_CS:
		std::cerr << "yaw left\n";
		if(IK_d.relHands) {
			Eigen::Vector3d rel(IK_d.hand[LEFT][XX]-IK_d.hand[RIGHT][XX], IK_d.hand[LEFT][YY]-IK_d.hand[RIGHT][YY], IK_d.hand[LEFT][ZZ]-IK_d.hand[RIGHT][ZZ]);
			Eigen::Quaterniond rot = EA2quat(0, 0, nrInc);
			rel = rot*rel;
			for(int i = 0; i < 3; i++)		IK_d.hand[LEFT][i] = IK_d.hand[RIGHT][i]+rel[i];
			for(int s = 0; s < 2; s++)		IK_d.handQ[s] = rot*IK_d.handQ[s];
		}
		else	for(int s = 0; s < 2; s++)	if(keyboardActive[s])	preModifyQ(IK_d.handQ[s], -nrInc,0,0);
		break;
	case atlas_ros_msgs::field_param::KEYBOARD_CF:
		std::cerr << "yaw right\n";
		if(IK_d.relHands) {
			Eigen::Vector3d rel(IK_d.hand[LEFT][XX]-IK_d.hand[RIGHT][XX], IK_d.hand[LEFT][YY]-IK_d.hand[RIGHT][YY], IK_d.hand[LEFT][ZZ]-IK_d.hand[RIGHT][ZZ]);
			Eigen::Quaterniond rot = EA2quat(0, 0, -nrInc);
			rel = rot*rel;
			for(int i = 0; i < 3; i++)		IK_d.hand[LEFT][i] = IK_d.hand[RIGHT][i]+rel[i];
			for(int s = 0; s < 2; s++)		IK_d.handQ[s] = rot*IK_d.handQ[s];
		}
		else	for(int s = 0; s < 2; s++)	if(keyboardActive[s])	preModifyQ(IK_d.handQ[s], nrInc,0,0);
		break;
	case atlas_ros_msgs::field_param::KEYBOARD_CE:
		std::cerr << "pitch up\n";
		if(IK_d.relHands) {
			Eigen::Vector3d rel(IK_d.hand[LEFT][XX]-IK_d.hand[RIGHT][XX], IK_d.hand[LEFT][YY]-IK_d.hand[RIGHT][YY], IK_d.hand[LEFT][ZZ]-IK_d.hand[RIGHT][ZZ]);
			Eigen::Quaterniond rot = EA2quat(0, -nrInc, 0);
			rel = rot*rel;
			for(int i = 0; i < 3; i++)		IK_d.hand[LEFT][i] = IK_d.hand[RIGHT][i]+rel[i];
			for(int s = 0; s < 2; s++)		IK_d.handQ[s] = rot*IK_d.handQ[s];
		}
		else	for(int s = 0; s < 2; s++)	if(keyboardActive[s])	preModifyQ(IK_d.handQ[s], 0,0,-nrInc);
		break;
	case atlas_ros_msgs::field_param::KEYBOARD_CD:
		std::cerr << "pitch down\n";
		if(IK_d.relHands) {
			Eigen::Vector3d rel(IK_d.hand[LEFT][XX]-IK_d.hand[RIGHT][XX], IK_d.hand[LEFT][YY]-IK_d.hand[RIGHT][YY], IK_d.hand[LEFT][ZZ]-IK_d.hand[RIGHT][ZZ]);
			Eigen::Quaterniond rot = EA2quat(0, nrInc, 0);
			rel = rot*rel;
			for(int i = 0; i < 3; i++)		IK_d.hand[LEFT][i] = IK_d.hand[RIGHT][i]+rel[i];
			for(int s = 0; s < 2; s++)		IK_d.handQ[s] = rot*IK_d.handQ[s];
		}
		else	for(int s = 0; s < 2; s++)	if(keyboardActive[s])	preModifyQ(IK_d.handQ[s], 0,0,nrInc);
		break;
	case atlas_ros_msgs::field_param::KEYBOARD_CW:
		std::cerr << "roll left\n";
		if(IK_d.relHands) {
			Eigen::Vector3d rel(IK_d.hand[LEFT][XX]-IK_d.hand[RIGHT][XX], IK_d.hand[LEFT][YY]-IK_d.hand[RIGHT][YY], IK_d.hand[LEFT][ZZ]-IK_d.hand[RIGHT][ZZ]);
			Eigen::Quaterniond rot = EA2quat(-nrInc, 0, 0);
			rel = rot*rel;
			for(int i = 0; i < 3; i++)		IK_d.hand[LEFT][i] = IK_d.hand[RIGHT][i]+rel[i];
			for(int s = 0; s < 2; s++)		IK_d.handQ[s] = rot*IK_d.handQ[s];
		}
		else	for(int s = 0; s < 2; s++)	if(keyboardActive[s])	preModifyQ(IK_d.handQ[s], 0,nrInc,0);
		break;
	case atlas_ros_msgs::field_param::KEYBOARD_CR:
		std::cerr << "roll right\n";
		if(IK_d.relHands) {
			Eigen::Vector3d rel(IK_d.hand[LEFT][XX]-IK_d.hand[RIGHT][XX], IK_d.hand[LEFT][YY]-IK_d.hand[RIGHT][YY], IK_d.hand[LEFT][ZZ]-IK_d.hand[RIGHT][ZZ]);
			Eigen::Quaterniond rot = EA2quat(nrInc, 0, 0);
			rel = rot*rel;
			for(int i = 0; i < 3; i++)		IK_d.hand[LEFT][i] = IK_d.hand[RIGHT][i]+rel[i];
			for(int s = 0; s < 2; s++)		IK_d.handQ[s] = rot*IK_d.handQ[s];
		}
		else	for(int s = 0; s < 2; s++)	if(keyboardActive[s])	preModifyQ(IK_d.handQ[s], 0,-nrInc,0);
		break;
	case atlas_ros_msgs::field_param::KEYBOARD_Q:
		std::cerr << "Neck Up\n";
		desNeck -= nrInc;
		break;
	case atlas_ros_msgs::field_param::KEYBOARD_A:
		std::cerr << "Neck Down\n";
		desNeck += nrInc;
		break;
	default:
		break;
	}
	keyboardCommand = -1;

	//relax excessive desired hand without moving IK position
	for(int s = 0; s < 2; s++) {
		for(int i = 0; i < 3; i++) {
			if((IK_d.hand[s][i] > IK.ikrs->hand[s][i]+relaxThresh) || (IK_d.hand[s][i] < IK.ikrs->hand[s][i]-relaxThresh))
				IK_d.hand[s][i] -= relaxGain*(IK_d.hand[s][i] - IK.ikrs->hand[s][i]);
		}
	}
}

void MBYlock() {
	IK_d.pelvQ = mySlerp(pelvQ0, pelvQf, ts/duration);
	Eigen::Vector3d pw = getAngVel(pelvQ0, pelvQf, duration);
	for(int i = 0; i < 3; i++)	IK_d.pelvW[i] = pw[i];
	IK_d.torsoQ = mySlerp(torsoQ0, torsoQf, ts/duration);
	Eigen::Vector3d tw = getAngVel(torsoQ0, torsoQf, duration);
	for(int i = 0; i < 3; i++)	IK_d.torsoW[i] = tw[i];


	if(ts > duration) {
		MBY_HARD_STOP = true;
		finishSeg = true;
		clearIKD();
	}
}

void MBYunlock() {
	IK_d.pelvQ = mySlerp(pelvQ0, pelvQf, ts/duration);
	Eigen::Vector3d pw = getAngVel(pelvQ0, pelvQf, duration);
	for(int i = 0; i < 3; i++)	IK_d.pelvW[i] = pw[i];
	IK_d.torsoQ = mySlerp(torsoQ0, torsoQf, ts/duration);
	Eigen::Vector3d tw = getAngVel(torsoQ0, torsoQf, duration);
	for(int i = 0; i < 3; i++)	IK_d.torsoW[i] = tw[i];

	if(ts > duration) {
		finishSeg = true;
		clearIKD();
	}
}


void directJoints() {
	if(ts > duration) {
		finishSeg = true;
		clearIKD();
		IK_d.rotGain[LEFT] = IK_d.rotGain[RIGHT] = &angGainStiff;
		IK_d.skipHand[djSide] = false;
		ID_d.skipHand[LEFT] = ID_d.skipHand[RIGHT] = false;
	}
}

void CoMshift() {
	IK_d.pelvQ = mySlerp(pelvQ0, pelvQf, ts/duration);
	Eigen::Vector3d pw = getAngVel(pelvQ0, pelvQf, duration);
	for(int i = 0; i < 3; i++)	IK_d.pelvW[i] = pw[i];
	IK_d.torsoQ = mySlerp(torsoQ0, torsoQf, ts/duration);
	Eigen::Vector3d tw = getAngVel(torsoQ0, torsoQf, duration);
	for(int i = 0; i < 3; i++)	IK_d.torsoW[i] = tw[i];

	CoMtraj[XX].read(ts, &(IK_d.com[XX]), &(IK_d.comd[XX]), &(comdd[XX]));
	CoMtraj[YY].read(ts, &(IK_d.com[YY]), &(IK_d.comd[YY]), &(comdd[YY]));
	CoMtraj[ZZ].read(ts, &(IK_d.root[ZZ]), &(IK_d.rootd[ZZ]), &(comdd[ZZ]));

	if(ts > duration) {
		finishSeg = true;
		clearIKD();
		IK_d.rotGain[LEFT] = IK_d.rotGain[RIGHT] = &angGainStiff;
	}
}

Eigen::Vector3d vRot, handPos;


void toTarget() {
	double theta;
	switch(trajMode) {
	case 1:	//circle mode
		theta = angleTraj.readPos(ts);
		//www.http://en.wikipedia.org/wiki/Rodrigues'_rotation_formula
		vRot = circleRadius*cos(theta) + circleAxis.cross(circleRadius)*sin(theta)+circleAxis*(circleAxis.dot(circleRadius))*(1-cos(theta));
		handPos = vRot+circleCenter;
		for(int i = 0; i < 3; i++)	IK_d.hand[circleSide][i] = handPos[i];
		break;
	default:	//regular & plunge
		for(int s = 0; s < 2; s++) {
			if(conHand[s])		IK_d.handQ[s] = mySlerp(handQ0[s], handQf[s], ts/duration);
			if(conHand[s])		for(int i = 0; i < 3; i++) 		handTraj[s][i].read(ts, &(IK_d.hand[s][i]), &(IK_d.handd[s][i]), &(handdd[s][i]));
		}
		break;
	}


	/*
	IK_d.pelvQ = mySlerp(pelvQ0, pelvQf, ts/duration);
	IK_d.torsoQ = mySlerp(torsoQ0, torsoQf, ts/duration);

	CoMtraj[XX].read(ts, &(IK_d.com[XX]), &(IK_d.comd[XX]), &(comdd[XX]));
	CoMtraj[YY].read(ts, &(IK_d.com[YY]), &(IK_d.comd[YY]), &(comdd[YY]));
	CoMtraj[ZZ].read(ts, &(IK_d.root[ZZ]), &(IK_d.rootd[ZZ]), &(comdd[ZZ]));
	 */

	if(ts > duration) {
		finishSeg = true;
	}
}



//variables for balanceCon
static double torsoGain_P[3] = {10, 10, 5};
static double torsoGain_D[3] = {1, 1, 1};
static double angMom_P[3] = {5, 5, 5};
static double comGain_P[3] = {10, 5, 5};
static double comGain_D[3] = {0.5, 0.5, 0.5};
static double handGain_P[6] = {5, 5, 5, 5, 5, 5};
static double handGain_D[6] = {1, 1, 1, 1, 1, 1};
static double footRotateGain[3] = {0.00, 0.00, 0};



void loadDrillParams() {
	std::string name = ros::package::getPath(pkg_name) + std::string("/cmu_config/con_param/EW_IKmanip.cf");
	std::ifstream in(name.c_str());
	IK.readParams(in);
	in.close();

	std::string name2 = ros::package::getPath(pkg_name) + std::string("/cmu_config/con_param/EW_IDmanip.cf");
	std::ifstream in2(name2.c_str());
	ID.readParams(in2);
	in2.close();

	std::string name3 = ros::package::getPath(pkg_name) + std::string("/cmu_config/con_param/EW_ManipBal.cf");
	std::ifstream in3(name3.c_str());

	if (in3.good()) {

		std::string name;
		std::map<const std::string, double *>::iterator res;
		double val;
		while (true)
		{
			in3 >> name;
			if (in3.eof())
				break;

			res = drillBalanceLookup.find(name);
			// can't find item
			if (res == drillBalanceLookup.end()) {
				std::cerr << "unknown param: " << name << " aborting." << std::endl;
				break;
			}

			in3 >> val;
			*(res->second) = val;
			std::cerr << "read " << name << " = " << val << std::endl;
		}
	}
	in3.close();
}

void balance2IKdrill(const PelvRobotState &rs, IDcmd &id_d, const PelvIkCon &ik) {
	ID_d.setZeros();
	ID_d.wl = 0.5;
	ID_d.contactState = DSc;

	//servo torso acceleration
	Eigen::Vector3d dAngT = quatMinus(rs.utorsoq, ik.ikrs->utorsoq);
	for(int i = 0; i < 3; i++)		id_d.torsoWd[i] = -torsoGain_P[i]*(dAngT[i]) - torsoGain_D[i]*(rs.utorsow[i]-ik.ikrs->utorsow[i]);

	//servo pelvis acceleration
	Eigen::Vector3d dAngP = quatMinus(rs.pelvisq, ik.ikrs->pelvisq);
	for(int i = 0; i < 3; i++)		id_d.pelvWd[i] = -torsoGain_P[i]*(dAngP[i]) -  2*sqrt(torsoGain_P[i])*torsoGain_D[i]*(rs.pelvisw[i]-ik.ikrs->pelvisw[i]);

	//try to reduce angular momentum (probably unused)
	for(int i = 0; i < 3; i++)		id_d.totMom[i] = -angMom_P[i]*rs.angMom[i];

	//set foot accelerations to 0.0 (redundant with setZeros above)
	for(int i = 0; i < LR; i++)		for(int j = 0; j < XYZ*2; j++)	id_d.footdd[i][j] = 0.0;

	//hand feedback
	for(int i = 0; i < LR; i++)	{
		//translation
		for(int j = 0; j < XYZ; j++) id_d.handdd[i][j] = handGain_P[j]*(ik.ikrs->hand[i][j]-rs.hand[i][j]) + handGain_D[j]*(ik.ikrs->handd[i][j]-rs.handd[i][j]);
		//orientation
		Eigen::Vector3d dAngH = quatMinus(rs.handQ[i], ik.ikrs->handQ[i]);
		for(int j = 0; j < 3; j++)		id_d.handdd[i][3+j] = -handGain_P[3+j]*(dAngH[j]) - handGain_D[3+j]*(rs.handd[i][3+j]-ik.ikrs->handd[i][3+j]);
	}

	double z = rs.com[ZZ] - (rs.feet[LEFT].w_sensor_pos[ZZ] + rs.feet[RIGHT].w_sensor_pos[ZZ]) / 2.;

	//com acceleration
	for(int i = 0; i < 3; i++) {
		id_d.comdd[i] = comGain_P[i]*(ik.ikrs->com[i] - rs.com[i]) - 2*sqrt(comGain_P[i])*comGain_D[i]*(rs.comd[i]-ik.ikrs->comd[i]);
	}
	if(id_d.comdd[YY] >  0.15)	id_d.comdd[YY] =  0.15;
	if(id_d.comdd[YY] < -0.15)	id_d.comdd[YY] = -0.15;


	//LIPM
	for(int i = 0; i < 2; i++)	id_d.cop[i] = rs.com[i] - id_d.comdd[i]*z/9.81;

}


void stateMachineDrill() {
	if(newCommand) {
		newCommand = false;
		sendMessageD("New Command!\n");
		if(commandInfo[0] == 5) {
			if(NcmdDat < 6)	sendMessageD("Need 6 joint angles\n");
			else if(NcmdInf < 2)	sendMessageD("Need to specify a side\n");
			else if(initDirectJoints()) {
				conType = 3;
				ts0 = tc;
			}
			else {
				sendMessageD("Command Rejected!");
				if(conType != 0) {
					conType = 0;
					initLockArms();
					ts0 = tc;
				}
			}
		}
		else if(commandInfo[0] == 11) {
			if(NcmdInf < 2) sendMessageD("Must specify a pose\n");
			else if(initPreDefinedArms(commandInfo[1])) {
				conType = 3;
				ts0 = tc;
			}
			else {
				sendMessageD("Command Rejected!");
				if(conType != 0) {
					conType = 0;
					initLockArms();
					ts0 = tc;
				}
			}
		}
		else if(commandInfo[0] == 6) {
			if(NcmdDat < 4) sendMessageD("Need at least 4 parameters\n");
			else if(initCoMshift()) {
				conType = 4;
				ts0 = tc;
			}
			else {
				sendMessageD("Command Rejected!");
				if(conType != 0) {
					conType = 0;
					initLockArms();
					ts0 = tc;
				}
			}
		}
		else if(commandInfo[0] == 8) {
			if(initMBYlock()) {
				conType = 5;
				ts0 = tc;
			}
			else {
				sendMessageD("Command Rejected!");
				if(conType != 0) {
					conType = 0;
					initLockArms();
					ts0 = tc;
				}
			}
		}
		else if(commandInfo[0] == 9) {
			if(initMBYunlock()) {
				conType = 6;
				ts0 = tc;
			}
			else {
				sendMessageD("Command Rejected!");
				if(conType != 0) {
					conType = 0;
					initLockArms();
					ts0 = tc;
				}
			}
		}
		else if(initToTargetFromCommand()) {
			sendMessageD("Command Accepted!\n");
			conType = 2;
			ts0 = tc;
		}
		else {
			sendMessageD("Command Rejected!");
			if(conType != 0) {
				conType = 0;
				initLockArms();
				ts0 = tc;
			}
		}
	}
	else if(finishSeg) {
		conType = 0;
		initLockArms();
		ts0 = tc;
	}

	finishSeg = false;
}


void doProcessing() {
	//integrate feet
	for(int f = 0; f < LR; f++) {	//both feet
		double EArs[3], EAik[3];
		quat2EA(IK_d.footQ[f], EAik);
		quat2EA(RS.feet[f].w_q, EArs);

		for(int d = 0; d < 3; d++) {		//x and y directions
			EAik[d] = EAik[d]*(1-footRotateGain[d])+EArs[d]*footRotateGain[d];
			if(d != 2)		limitMag(EAik[d], 0.07);

			IK_d.foot[f][d] += footIntTimeConst*(RS.feet[f].w_sensor_pos[d] - IK_d.foot[f][d]);
		}
		IK_d.footQ[f] = EA2quat(EAik);
	}

	//comOffset
	//comIntX += (RS.cop[XX]-ID.cop[XX])*1/(333.0*comIntTimeConst);
	limitMag(comIntX, comIntLim);

	IK.ikrs->comOffset[XX] = RS.comOffset[XX] = comIntX;


	//center com
	Eigen::Vector3d fc = IK.ikrs->getFootCenter();
	IK_d.com[XX] += (fc[XX]+home[XX]-IK_d.com[XX])*homeGain[XX];
	IK_d.com[YY] += (fc[YY]+home[YY]-IK_d.com[YY])*homeGain[YY];
	IK_d.root[ZZ] += (fc[ZZ]+home[ZZ]-IK_d.root[ZZ])*homeGain[ZZ];
	IK_d.rootMode[ZZ] = true;

	if(MBY_HARD_STOP) {
		homePelvis = pelvisBack*homeTorso;
		IK.hardenTorso = false;
		jointsByTraj[1] = true;
		jointTraj[1].setConstant(MBYlockAng);
	}
	else {
		IK.hardenTorso = true;
		homePelvis = homeTorso;
	}

	IK_d.pelvQ = mySlerp(IK_d.pelvQ, homePelvis, homeTorsoGain);
	IK_d.torsoQ = mySlerp(IK_d.torsoQ, homeTorso, homeTorsoGain);


	for(int s = 0; s < 2; s++) 	for(int i = 0; i < 3; i++)  handdd[s][i] = 0.0;
	for(int i = 0; i < 3; i++)								comdd[i] = 0.0;

	switch(conType) {
	case 1:
		goToUpperPose();
		break;
	case 2:
		toTarget();
		break;
	case 3:
		directJoints();
		break;
	case 4:
		CoMshift();
		break;
	case 5:
		MBYlock();
		break;
	case 6:
		MBYunlock();
		break;
	default:
		lockArms();
		break;
	}

	//neck hack
	jointTraj[3].setConstant(desNeck);
	jointsByTraj[3] = true;

	//handle fixed joints
	double jtd[PELV_N_JOINTS];
	for(int i = 0; i < 28; i++) {
		if(jointsByTraj[i]) {
			jointTraj[i].read(ts, &(IK.forceQ[i]), &(jtd[i]), &(ID.fixQdd[i+6]));
			IK.forceMask[i] = true;
			ID.fixQDDmask[i+6] = true;
		}
		else {
			IK.forceMask[i] = false;
			ID.fixQDDmask[i+6] = false;
		}
	}

	IK.IK(IK_d);

	//cleans up noise
	for(int i = 0; i < 28; i++) 	if(jointsByTraj[i]) 	IK.ikrs->jointsd[i] = jtd[i];

	balance2IKdrill(RS, ID_d, IK);
	for(int s = 0; s < 2; s++)		for(int i = 0; i < 3; i++)		ID_d.handdd[s][i] += handdd[s][i];
	for(int i = 0; i < 3; i++)										ID_d.comdd[i] += comdd[i];

	ID.ID(RS, ID_d);
}


void control_loop_ew_manip(const atlas_msgs::AtlasState &data_from_robot,
    boost::mutex &data_from_robot_lock,
    atlas_msgs::AtlasCommand &data_to_robot,
    atlas_ros_msgs::field_param &param,
    atlas_ros_msgs::simple_text *to_ocu,
    bool firstTime) 
{
  txt = to_ocu;
	realMessageBuf[0] = '\0';

	IKDclearedAlready = false;

	if(param.cmd.size() >= 1) {	//if any message received
		sprintf(messageBuf, "Receive size (%d, %d)\n", param.cmd.size(), param.param.size());
		sendMessageD();
		for(int i = 0; i < param.cmd.size(); i++) {
			sprintf(messageBuf, "%d ", param.cmd[i]);
			sendMessageD();
		}
		for(int i = 0; i < param.param.size(); i++) {
			sprintf(messageBuf, "%g ", param.param[i]);
			sendMessageD();
		}

		if(param.cmd[0] == 4) {
			loadDrillParams();
		}
		else if(param.cmd[0] == 2) {
			sendMessageD("Freezing!\n");
			clearIKD();
			conType = 0;
			initLockArms();
			ts0 = tc;
		}
		else if(param.cmd[0] == 13) {
			if(param.param.size() >= 1) {
				ntInc = param.param[0];
				sprintf(messageBuf, "Setting translational increment to %g\n", ntInc);
				sendMessageD();
			}
			else {
				sendMessageD("You must specify an increment\n");
			}
		}
		else if(param.cmd[0] == 14) {
			if(param.param.size() >= 1) {
				nrInc = param.param[0];
				sprintf(messageBuf, "Setting rotational increment to %g\n", nrInc);
				sendMessageD();
			}
			else {
				sendMessageD("You must specify an increment\n");
			}
		}
		else if(param.cmd[0] == 12) {
			if(param.cmd.size() >= 2) {
				clearIKD();
				if(param.cmd[1]==0) {
					if(IK_d.drillAxisMode == false) {
						sendMessageD("Already in regular hand axis mode\n");
					}
					else {
						IK_d.drillAxisMode = false;
						sendMessageD("Setting to regular hand axis mode\n");
					}
				}
				else {
					if(IK_d.drillAxisMode == true) {
						sendMessageD("Already in drill hand axis mode\n");
					}
					else {
						IK_d.drillAxisMode = true;
						sendMessageD("Setting to drill hand axis mode\n");
					}
				}
			}
			else {
				sendMessageD("You must specify a mode\n");
			}
		}
		else if(param.cmd[0] == 17) {	//keyboard command
			sprintf(messageBuf, "keyboard: %d\n",param.cmd[2]);
			sendMessageD();
			keyboardCommand = param.cmd[2];
			if(param.cmd[1] == 10 && keyboardActive[LEFT] == false) {
				if(IK_d.relHands) 		sendMessageD("Can't switch to left from two hands\n");
				else if(IK_d.skipHand[LEFT])	sendMessageD("Can't switch to left in lock joint mode\n");
				else {
					sendMessageD("Switch to left hand active\n");
					keyboardActive[LEFT] = true;
					keyboardActive[RIGHT] = false;
				}
			}
			if(param.cmd[1] == 11 && keyboardActive[RIGHT] == false) {
				if(IK_d.relHands) 		sendMessageD("Can't switch to right from two hands\n");
				else if(IK_d.skipHand[RIGHT])	sendMessageD("Can't switch to right in lock joint mode\n");
				else {
					sendMessageD("Switch to right hand active\n");
					keyboardActive[RIGHT] = true;
					keyboardActive[LEFT] = false;
				}
			}
		}
		else if(param.cmd[0] == 7)	{
			if(param.param.size() >= 1) {
				desNeck = param.param[0];
				sprintf(messageBuf, "Setting neck to %g\n", param.param[0]);
				sendMessageD();
			}
			else		sendMessageD("No neck angle given\n");
		}
		else if(param.cmd[0] == 10) {
			if(param.param.size() >= 1) {	//if we have a parameter
				PelvRobotState::setHandDist(param.param[0]);
				RS.computeSDFvars();
				IK.ikrs->computeSDFvars();
				clearIKD();
				sprintf(messageBuf, "Setting hand distance to %g\n", param.param[0]);
				sendMessageD();
			}
			else		sendMessageD("No distance given for setting hand distance\n");
		}
		else {
			for(int i = 0; i < param.param.size(); i++) 				commandData[i] = param.param[i];
			for(int i = 0; i < param.cmd.size(); i++) 					commandInfo[i] = param.cmd[i];
			NcmdDat = param.param.size();
			NcmdInf = param.cmd.size();
			newCommand = true;
		}
		param.cmd.clear();
	}
	tc = RS.time-tc0;
	ts = tc-ts0;

  // unpack data and do state estimation
	int cState = DSc;
  if (firstTime) {
    pre_enable_ew_manip(data_from_robot, data_from_robot_lock);
    cmu_utils.estimateStateEric(cState, kcekf, cmu_utils.foot_forces[LEFT][ZZ], cmu_utils.foot_forces[RIGHT][ZZ]);
    cmu_utils.updateRobotState(cState, RS);
  }
  else {
    {
      boost::mutex::scoped_lock lock(data_from_robot_lock);
      cmu_utils.UnpackDataFromRobot(data_from_robot, initYaw);
    }
    cmu_utils.estimateStateEric(cState, kcekf, cmu_utils.foot_forces[LEFT][ZZ], cmu_utils.foot_forces[RIGHT][ZZ]);
    cmu_utils.updateRobotState(cState, RS);
  }
	
	cmu_utils.setArmMode(CMUCtrlUtils::PD);
	cmu_utils.setLegMode(CMUCtrlUtils::PD_FF);
	cmu_utils.setSpineMode(CMUCtrlUtils::PD_FF);

	stateMachineDrill();
	ts = tc-ts0;
	doProcessing();

	// stuff

	double tau_d[N_JOINTS]={0.0};
	double theta_d[N_JOINTS]={0.0};
	double thetad_d[N_JOINTS]={0.0};
	ID.getCommand(tau_d);
	IK.getCommand(theta_d, thetad_d);



	cmu_utils.PackDataToRobot(tau_d, theta_d, thetad_d, data_to_robot);

	logger.saveData();
}

void initialize_loop_ew_manip()
{
	drillBalanceLookup["TORSO_GAIN_P_X"] = &(torsoGain_P[XX]);
	drillBalanceLookup["TORSO_GAIN_P_Y"] = &(torsoGain_P[YY]);
	drillBalanceLookup["TORSO_GAIN_P_Z"] = &(torsoGain_P[ZZ]);

	drillBalanceLookup["TORSO_GAIN_D_X"] = &(torsoGain_D[XX]);
	drillBalanceLookup["TORSO_GAIN_D_Y"] = &(torsoGain_D[YY]);
	drillBalanceLookup["TORSO_GAIN_D_Z"] = &(torsoGain_D[ZZ]);

	drillBalanceLookup["COM_GAIN_P_X"] = &(comGain_P[XX]);
	drillBalanceLookup["COM_GAIN_P_Y"] = &(comGain_P[YY]);
	drillBalanceLookup["COM_GAIN_P_Z"] = &(comGain_P[ZZ]);

	drillBalanceLookup["COM_GAIN_D_X"] = &(comGain_D[XX]);
	drillBalanceLookup["COM_GAIN_D_Y"] = &(comGain_D[YY]);
	drillBalanceLookup["COM_GAIN_D_Z"] = &(comGain_D[ZZ]);

	drillBalanceLookup["FOOT_ROT_I_ROLL"] = &(footRotateGain[0]);
	drillBalanceLookup["FOOT_ROT_I_PITCH"] = &(footRotateGain[1]);
	drillBalanceLookup["FOOT_ROT_I_YAW"] = &(footRotateGain[2]);

	drillBalanceLookup["RELAX_THRESH"] = &(relaxThresh);
	drillBalanceLookup["RELAX_GAIN"] = &(relaxGain);

	drillBalanceLookup["HOME_X_GAIN"] = &(homeGain[XX]);
	drillBalanceLookup["HOME_Y_GAIN"] = &(homeGain[YY]);
	drillBalanceLookup["HOME_Z_GAIN"] = &(homeGain[ZZ]);

	drillBalanceLookup["MBY_LOCK_TIME"] = &MBYlockTime;
	drillBalanceLookup["MBY_UNLOCK_TIME"] = &MBYunlockTime;

	drillBalanceLookup["FOOT_INT_TIME_CONSTANT"] = &footIntTimeConst;
	drillBalanceLookup["COM_INT_TIME_CONSTANT"] = &comIntTimeConst;
	drillBalanceLookup["COM_INT_LIMIT"] = &comIntLim;

	loadDrillParams();

	//load_KFParams(kcekf);
	load_KFEricParams(std::string("test_walk"), kcekf);

  logger.init(TIME_STEP);
	logger.addEWstatic();
	logger.add_datapoint("time","s",&tc);
	logger.add_datapoint("ts","s",&ts);
	logger.add_datapoint("conType","-",&conType);
	logger.add_datapoint("step","-",&step);
	logger.add_datapoint("duration","s",&duration);

	logger.add_datapoint("home[X]","m",&(home[XX]));
	logger.add_datapoint("home[Y]","m",&(home[YY]));
	logger.add_datapoint("home[Z]","m",&(home[ZZ]));
	logger.add_datapoint("CoMint[X]","m",&comIntX);

	RS.addToLog(logger);
	IK_d.addToLog(logger);
	IK.addToLog(logger);
	ID_d.addToLog(logger);
	ID.addToLog(logger);
	cmu_utils.addToLog(logger);
}

// This function is called when the pump stops
void quit_ew_manip()
{
	logger.writeToMRDPLOT();
}

// Called each time the controller is enabled
static void pre_enable_ew_manip(const atlas_msgs::AtlasState &data_from_robot, 
    boost::mutex &data_from_robot_lock)    
{
	PelvRobotState::setHandDist(0.29);
  {
    boost::mutex::scoped_lock lock(data_from_robot_lock);
    cmu_utils.UnpackDataFromRobot(data_from_robot);
    initYaw = getYaw(Eigen::Map<Eigen::Quaterniond>(cmu_utils.imu_orientation));
    fprintf(stderr, "yaw init (drill) %g\n", initYaw);

    cmu_utils.UnpackDataFromRobot(data_from_robot, initYaw);
  }

	RS.time = cmu_utils.time;
	tc0 = RS.time;
	
	// init filter stuff
	//cmu_utils.init_KF(kcekf, RS.getType(), 0);
	cmu_utils.init_KFEric(kcekf, RS.getType(), 0);
	cmu_utils.updateRobotState(DSc, RS); 
	double z = -0.5*(RS.feet[LEFT].w_sensor_pos[ZZ] + RS.feet[RIGHT].w_sensor_pos[ZZ]);
	//cmu_utils.init_KF(kcekf, RS.getType(), z);
	cmu_utils.init_KFEric(kcekf, RS.getType(), z);
	cmu_utils.updateRobotState(DSc, RS);

	if(isPrevCommandSet(RS.time))  {
		double *prevTheta_d = getPrevCommand();
		for(int i = 0; i < 28; i++)	{
			IK.ikrs->joints[i] = prevTheta_d[i];
			IK.ikrs->jointsd[i] = 0.0;
			IK.matchRoot(RS);
		}
	}
	else {
		IK.setToRobotState(RS);
		IK.matchRoot(RS);
	}
	clearIKD();
	IK_d.constrainCoM = true;
	IK.restoreDefaults();
	ID.restoreDefaults();
	//IK_d.root[ZZ] = 0.9;

	desNeck = RS.joints[3];

	ts0 = 0;

	if(RS.joints[7] > 0.25 && RS.joints[13] > 0.25)		IK_d.noLockKnees = true;
	else												IK_d.noLockKnees = false;

	//start with demo motion
	demoArmScript();
	conType = 1;

	home[0] = 0.045;
	home[1] = 0.0;
	home[2] = 0.9;
	comIntX = 0.0;
	keyboardCommand = -1;
	keyboardActive[LEFT] = keyboardActive[RIGHT] = false;
	IKDclearedAlready=false;
}
