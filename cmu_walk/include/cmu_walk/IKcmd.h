#ifndef IKCMD_INCLUDED
#define IKCMD_INCLUDED

#include "Foot.h"
#include "drc_common_defines.h"
#include <eigen3/Eigen/Geometry>
#include "Logger.h"
#include "RobotState.h"
#include "Eigen_utils.hpp"

class IKcmd {
public:
	Foot::FootRefType footRef[LR];
	double com_offset[3];

	double com[3], comd[3];
	double root[3], rootd[3];
	//double root_b_w[3];
	bool rootMode[3];
	Eigen::Quaterniond pelvQ, torsoQ;
	double pelvW[3], torsoW[3];

	double foot[2][3];
	Eigen::Quaterniond footQ[2];
	double footd[2][6];

	double hand[2][3];
	Eigen::Quaterniond handQ[2];
	double handd[2][6];

	double elbowLims[2][3];
	double desElbow[2];
	double elbowWeight[2];

	// desired angles
	double joints[N_JOINTS];
	double jointsd[N_JOINTS];
	bool stanceFoot[2];
	bool skipFoot[2];
	bool skipHand[2];

	bool dropHeight;
	bool constrainCoM;
	bool relHands;
	bool noLockKnees;

	bool drillAxisMode;
	const Eigen::Vector3d *rotGain[2];


public:
	IKcmd() {
		com_offset[0] = com_offset[1] = com_offset[2] = 0;

		footRef[LEFT] = footRef[RIGHT] = Foot::RefSensor;
		elbowLims[0][0] = elbowLims[1][0] = 0.0;
		elbowLims[0][1] = elbowLims[1][1] = 0.0;
		elbowLims[0][2] = elbowLims[1][2] = 0.0;
		desElbow[LEFT] = desElbow[RIGHT] = 0.0;
		elbowWeight[LEFT] = elbowWeight[RIGHT] = 0.0;
		for(int i = 0; i < 3; i++)	rootMode[i] = false;
		stanceFoot[0] = stanceFoot[1] = false;
		skipFoot[0] = skipFoot[1] = false;
		skipHand[0] = skipHand[1] = false;
		dropHeight = false;
		rotGain[LEFT] = rotGain[RIGHT] = NULL;
		constrainCoM = false;
		relHands = false;
		noLockKnees = false;
		drillAxisMode = false;
	}

	void yawPt(const double *center, double *pt, double yaw) {
		double rel[2] = {pt[XX]-center[XX], pt[YY]-center[YY]};
		double rotate[2] = {cos(yaw)*rel[XX]-sin(yaw)*rel[YY], cos(yaw)*rel[YY]+sin(yaw)*rel[XX]};

		pt[XX] = center[XX]+rotate[XX];
		pt[YY] = center[YY]+rotate[YY];
	}


	void yawAboutPt(const double *pt, double yaw) {
		yawPt(pt, com, yaw);
		yawPt(pt, root, yaw);
		yawPt(pt, &(hand[LEFT][0]), yaw);
		yawPt(pt, &(hand[RIGHT][0]), yaw);
		yawPt(pt, &(foot[LEFT][0]), yaw);
		yawPt(pt, &(foot[RIGHT][0]), yaw);

		yawQuat(pelvQ, yaw);
		yawQuat(torsoQ, yaw);
		yawQuat(footQ[LEFT], yaw);
		yawQuat(footQ[RIGHT], yaw);
		yawQuat(handQ[LEFT], yaw);
		yawQuat(handQ[RIGHT], yaw);
	}

	void setToRS(const PelvRobotState &rs) {
		for(int i = 0; i < 3; i++) {
			com[i] = rs.com[i];
			root[i] = rs.root[i];
			comd[i] = rs.comd[i];
			rootd[i] = rs.rootd[i];
			pelvW[i] = rs.pelvisw[i];
			torsoW[i] = rs.utorsow[i];
		}
		pelvQ = rs.pelvisq;
		torsoQ = rs.utorsoq;

		for(int i = 0; i < 2; i++) {
			footQ[i] = rs.feet[i].w_q;
			handQ[i] = rs.handQ[i];
			for(int j = 0; j < 3; j++) {
				hand[i][j] = rs.hand[i][j];
				foot[i][j] = rs.feet[i].w_sensor_pos[j];
			}
			for(int j = 0; j < 6; j++) {
				footd[i][j] = rs.feet[i].w_sensor_vel[j];
				handd[i][j] = rs.handd[i][j];
			}
		}
	}

	void setVel0() {
		for(int i = 0; i < 3; i++) {
			comd[i] = 0.0;
			pelvW[i] = 0.0;
			torsoW[i] = 0.0;
		}
		for(int i = 0; i < 6; i++) {
			for(int j = 0; j < 2; j++) {
				handd[j][i] = 0.0;
				footd[j][i] = 0.0;
			}
		}
	}

	void addToLog(BatchLogger &logger) {
		logger.add_datapoint("IK_d.footRef[L]","m", (const int *)&(footRef[LEFT]));
		logger.add_datapoint("IK_d.footRef[R]","m", (const int *)&(footRef[RIGHT]));

		logger.add_datapoint("IK_d.com_offset[X]","m",&(com_offset[XX]));
		logger.add_datapoint("IK_d.com_offset[Y]","m",&(com_offset[YY]));
		logger.add_datapoint("IK_d.com_offset[Z]","m",&(com_offset[ZZ]));

		logger.add_datapoint("IK_d.com[X]","m",&(com[XX]));
		logger.add_datapoint("IK_d.com[Y]","m",&(com[YY]));
		logger.add_datapoint("IK_d.com[Z]","m",&(com[ZZ]));
		logger.add_datapoint("IK_d.comd[X]","v",&(comd[XX]));
		logger.add_datapoint("IK_d.comd[Y]","v",&(comd[YY]));
		logger.add_datapoint("IK_d.comd[Z]","v",&(comd[ZZ]));

		logger.add_quat("IK_d.pelvQ",&(pelvQ));
		logger.add_quat("IK_d.torsoQ",&(torsoQ));

		logger.add_datapoint("IK_d.pelvW[X]","-",&(pelvW[XX]));
		logger.add_datapoint("IK_d.pelvW[Y]","-",&(pelvW[YY]));
		logger.add_datapoint("IK_d.pelvW[Z]","-",&(pelvW[ZZ]));
		logger.add_datapoint("IK_d.torsoW[X]","-",&(torsoW[XX]));
		logger.add_datapoint("IK_d.torsoW[Y]","-",&(torsoW[YY]));
		logger.add_datapoint("IK_d.torsoW[Z]","-",&(torsoW[ZZ]));

		logger.add_datapoint("IK_d.root[X]","m",&(root[XX]));
		logger.add_datapoint("IK_d.root[Y]","m",&(root[YY]));
		logger.add_datapoint("IK_d.root[Z]","m",&(root[ZZ]));
		logger.add_datapoint("IK_d.rootd[X]","v",&(rootd[XX]));
		logger.add_datapoint("IK_d.rootd[Y]","v",&(rootd[YY]));
		logger.add_datapoint("IK_d.rootd[Z]","v",&(rootd[ZZ]));

		logger.add_datapoint("IK_d.foot[L][X]","m",&(foot[LEFT][XX]));
		logger.add_datapoint("IK_d.foot[L][Y]","m",&(foot[LEFT][YY]));
		logger.add_datapoint("IK_d.foot[L][Z]","m",&(foot[LEFT][ZZ]));

		logger.add_datapoint("IK_d.foot[R][X]","m",&(foot[RIGHT][XX]));
		logger.add_datapoint("IK_d.foot[R][Y]","m",&(foot[RIGHT][YY]));
		logger.add_datapoint("IK_d.foot[R][Z]","m",&(foot[RIGHT][ZZ]));

		logger.add_datapoint("IK_d.footd[L][X]","v",&(footd[LEFT][XX]));
		logger.add_datapoint("IK_d.footd[L][Y]","v",&(footd[LEFT][YY]));
		logger.add_datapoint("IK_d.footd[L][Z]","v",&(footd[LEFT][ZZ]));

		logger.add_datapoint("IK_d.footd[R][X]","v",&(footd[RIGHT][XX]));
		logger.add_datapoint("IK_d.footd[R][Y]","v",&(footd[RIGHT][YY]));
		logger.add_datapoint("IK_d.footd[R][Z]","v",&(footd[RIGHT][ZZ]));

		logger.add_quat("IK_d.footQ[L]",&(footQ[LEFT]));
		logger.add_quat("IK_d.footQ[R]",&(footQ[RIGHT]));

		logger.add_datapoint("IK_d.footw[L][X]","v",&(footd[LEFT][3+XX]));
		logger.add_datapoint("IK_d.footw[L][Y]","v",&(footd[LEFT][3+YY]));
		logger.add_datapoint("IK_d.footw[L][Z]","v",&(footd[LEFT][3+ZZ]));

		logger.add_datapoint("IK_d.footw[R][X]","v",&(footd[RIGHT][3+XX]));
		logger.add_datapoint("IK_d.footw[R][Y]","v",&(footd[RIGHT][3+YY]));
		logger.add_datapoint("IK_d.footw[R][Z]","v",&(footd[RIGHT][3+ZZ]));

		logger.add_datapoint("IK_d.hand[L][X]","m",&(hand[LEFT][XX]));
		logger.add_datapoint("IK_d.hand[L][Y]","m",&(hand[LEFT][YY]));
		logger.add_datapoint("IK_d.hand[L][Z]","m",&(hand[LEFT][ZZ]));

		logger.add_datapoint("IK_d.hand[R][X]","m",&(hand[RIGHT][XX]));
		logger.add_datapoint("IK_d.hand[R][Y]","m",&(hand[RIGHT][YY]));
		logger.add_datapoint("IK_d.hand[R][Z]","m",&(hand[RIGHT][ZZ]));

		logger.add_datapoint("IK_d.handd[L][X]","v",&(handd[LEFT][XX]));
		logger.add_datapoint("IK_d.handd[L][Y]","v",&(handd[LEFT][YY]));
		logger.add_datapoint("IK_d.handd[L][Z]","v",&(handd[LEFT][ZZ]));

		logger.add_datapoint("IK_d.handd[R][X]","v",&(handd[RIGHT][XX]));
		logger.add_datapoint("IK_d.handd[R][Y]","v",&(handd[RIGHT][YY]));
		logger.add_datapoint("IK_d.handd[R][Z]","v",&(handd[RIGHT][ZZ]));

		logger.add_quat("IK_d.handQ[L]",&(handQ[LEFT]));
		logger.add_quat("IK_d.handQ[R]",&(handQ[RIGHT]));

		logger.add_datapoint("IK_d.elbow[L][A]","m",&(elbowLims[0][0]));
		logger.add_datapoint("IK_d.elbow[L][B]","m",&(elbowLims[0][1]));
		logger.add_datapoint("IK_d.elbow[L][C]","m",&(elbowLims[0][2]));

		logger.add_datapoint("IK_d.elbow[R][A]","m",&(elbowLims[1][0]));
		logger.add_datapoint("IK_d.elbow[R][B]","m",&(elbowLims[1][1]));
		logger.add_datapoint("IK_d.elbow[R][C]","m",&(elbowLims[1][2]));

		logger.add_datapoint("IK_d.elbowW[L]","-",&(elbowWeight[LEFT]));
		logger.add_datapoint("IK_d.elbowW[R]","-",&(elbowWeight[RIGHT]));

		logger.add_datapoint("IK_d.desElbow[L]","-",&(desElbow[LEFT]));
		logger.add_datapoint("IK_d.desElbow[R]","-",&(desElbow[RIGHT]));

		logger.add_datapoint("IK_d.rootMode[X]","-",&(rootMode[XX]));
		logger.add_datapoint("IK_d.rootMode[Y]","-",&(rootMode[YY]));
		logger.add_datapoint("IK_d.rootMode[Z]","-",&(rootMode[ZZ]));
		logger.add_datapoint("IK_d.dropHeight","-",&dropHeight);

		logger.add_datapoint("IK_d.skipFoot[L]","-", &(skipFoot[LEFT]));
		logger.add_datapoint("IK_d.skipFoot[R]","-", &(skipFoot[RIGHT]));
		logger.add_datapoint("IK_d.skipHand[L]","-", &(skipHand[LEFT]));
		logger.add_datapoint("IK_d.skipHand[R]","-", &(skipHand[RIGHT]));

		logger.add_datapoint("IK_d.relHands","-", &(relHands));
		logger.add_datapoint("IK_d.daMode","-", &(drillAxisMode));

		logger.add_datapoint("IK_d.stanceFoot[L]","-", &(stanceFoot[LEFT]));
		logger.add_datapoint("IK_d.stanceFoot[R]","-", &(stanceFoot[RIGHT]));

		char buf[1000];
		for (int i = 0; i < N_JOINTS; i++) {
			sprintf(buf,"IK_d.%s", RobotState::joint_names[i].c_str());
			logger.add_datapoint(buf, "rad", &(joints[i]));
		}
		for (int i = 0; i < N_JOINTS; i++) {
			sprintf(buf,"IK_d.%sd", RobotState::joint_names[i].c_str());
			logger.add_datapoint(buf, "rad/s", &(jointsd[i]));
		}
	}

};

#endif
