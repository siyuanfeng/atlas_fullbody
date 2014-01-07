#include "RobotState.h"
#include "drc_pelvis.h"
#include "drc_pelvis_defines.h"
#include "SDModel_common.h"
#include "Eigen_utils.hpp"
#include "Utils.hpp"

PelvRobotState::PelvRobotState()
{
	for(int i = 0; i < 3; i++)		comOffset[i] = 0.0;

	_type = RobotState::TYPE_PELVIS;
	_sdFastState = new double[PELV_N_SDFAST_STATE];
	model = new drc_pelvis();
	model->sdinit();

	for (int i = 0; i < LR; i++) {
		J[i] = Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor>::Zero(PELV_N_SDFAST_U,6);
		Jhand[i] = Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor>::Zero(PELV_N_SDFAST_U,6);
		Jd[i] = Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor>::Zero(PELV_N_SDFAST_U,6);
		Jtoe[i] = Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor>::Zero(PELV_N_SDFAST_U,6);
		Jtoed[i] = Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor>::Zero(PELV_N_SDFAST_U,6);
		Jheel[i] = Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor>::Zero(PELV_N_SDFAST_U,6);
		Jheeld[i] = Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor>::Zero(PELV_N_SDFAST_U,6);
		Jmid[i] = Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor>::Zero(PELV_N_SDFAST_U,6);
		Jmidd[i] = Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor>::Zero(PELV_N_SDFAST_U,6);

		Jhandd[i] = Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor>::Zero(PELV_N_SDFAST_U,6);
		Jelbow[i] = Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor>::Zero(PELV_N_SDFAST_U,6);
	}

	Jc = Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor>::Zero(PELV_N_SDFAST_U,3);
	Jcd = Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor>::Zero(PELV_N_SDFAST_U,3);

	Jtorso = Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor>::Zero(PELV_N_SDFAST_U,6);
	Jtorsod = Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor>::Zero(PELV_N_SDFAST_U,6);
	Jpelvis = Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor>::Zero(PELV_N_SDFAST_U,6);
	Jpelvisd = Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor>::Zero(PELV_N_SDFAST_U,6);

	M = Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor>::Zero(PELV_N_SDFAST_U,PELV_N_SDFAST_U);
	nonLin = Eigen::Matrix<double,Eigen::Dynamic,1>::Zero(PELV_N_SDFAST_U);
}

PelvRobotState::~PelvRobotState()
{
	delete []_sdFastState;
	delete model;
}

double wrist2hand[2][3] = {{0, 0.3, 0}, {0, -0.3, 0}};

void PelvRobotState::setHandDist(double d) {
	wrist2hand[LEFT][YY] = d;
	wrist2hand[RIGHT][YY] = -d;
}

void PelvRobotState::setHandDist(double d, int side) {
	if(side == LEFT)	wrist2hand[LEFT][YY] = d;
	else				wrist2hand[RIGHT][YY] = -d;
}

void PelvRobotState::computeSDFvars()
{
	makeSDFastState(root, rootq, rootd, root_b_w, joints, jointsd, _sdFastState);
	set_state(model, _sdFastState);

	double temp[6];
	double zeros[3] = {0, 0, 0};
	const int joint[2] = {PELV_S_JOINT_L_LEG_LAX, PELV_S_JOINT_R_LEG_LAX};
	const int body[2] = {PELV_S_BODY_L_FOOT, PELV_S_BODY_R_FOOT};
	const int handBodyInd[2] = {PELV_S_BODY_L_HAND, PELV_S_BODY_R_HAND};
	const int elbowBodyInd[2] = {PELV_S_BODY_L_LARM, PELV_S_BODY_R_LARM};
	Eigen::Vector3d handAxis0[2] = {Eigen::Vector3d(0, 1, 0), Eigen::Vector3d(0, -1, 0)};
	Eigen::Vector3d drillAxis0[2] = {Eigen::Vector3d(0, 0, 1), Eigen::Vector3d(0, 0, 1)};

	//momentem and KE
	model->sdmom(mom, angMom, &KE);
	double dircos[3][3];
	double qq[4];
	double com_to_ankle[3];

	dvec_copy(temp, root2tfRoot, 3);
	model->sdpos(PELV_S_BODY_PELVIS, temp, tfRoot);

	for (int side = 0; side < 2; side++) {
		model->sdgetbtj(joint[side], com_to_ankle);

		//foot positions and jacobians
		dvec_add(com_to_ankle, Foot::ankle_to_sensor_offset, temp, 3);
		body_position(model, NULL, body[side], temp, feet[side].w_sensor_pos, J[side].data());
		model->sdvel(body[side], temp, feet[side].w_sensor_vel);

		//mid of foot pos and vel
		dvec_add(com_to_ankle, feet[side].ankle_to_center_offset, temp, 3);
		body_position(model, NULL, body[side], temp, feet[side].w_mid_pos, Jmid[side].data());
		model->sdvel(body[side], temp, feet[side].w_mid_vel);

		//toe pos and vel
		dvec_add(com_to_ankle, feet[side].ankle_to_toe_offset, temp, 3);
		body_position(model, NULL, body[side], temp, feet[side].w_toe_pos, Jtoe[side].data());
		model->sdvel(body[side], temp, feet[side].w_toe_vel);

		//heel pos and vel
		dvec_add(com_to_ankle, feet[side].ankle_to_heel_offset, temp, 3);
		body_position(model, NULL, body[side], temp, feet[side].w_heel_pos, Jheel[side].data());
		model->sdvel(body[side], temp, feet[side].w_heel_vel);

		//hand position (translation) and Jacobian
		double handOffset[3];
		model->sdgetbtj(handBodyInd[side], handOffset);
		for(int i = 0; i < 3; i++)	handOffset[i] += wrist2hand[side][i];
		body_position(model, NULL, handBodyInd[side], handOffset, &(hand[side][0]), Jhand[side].data());

		double elbowOffset[3];
		model->sdgetbtj(elbowBodyInd[side], elbowOffset);
		body_position(model, NULL, elbowBodyInd[side], elbowOffset, &(elbow[side][0]), Jelbow[side].data());

		//foot quaternion
		model->sdorient(body[side], dircos);
		model->sddc2quat(dircos, qq, qq+1, qq+2, qq+3);
		feet[side].w_q = Eigen::Quaterniond(qq[3], qq[0], qq[1], qq[2]).normalized();

		//hand quaternion
		model->sdorient(handBodyInd[side], dircos);
		model->sddc2quat(dircos, qq, qq+1, qq+2, qq+3);
		handQ[side] = Eigen::Quaterniond(qq[3], qq[0], qq[1], qq[2]).normalized();
		handAxis[side] = handQ[side]*handAxis0[side];
		drillAxis[side] = handQ[side]*drillAxis0[side];

		//hand velocities
		model->sdvel(handBodyInd[side], handOffset, &(handd[side][0]));
		//foot angular velocities
		model->sdangvel(body[side], feet[side].b_w);
		model->sdtrans(body[side], feet[side].b_w, -1, &(feet[side].w_sensor_vel[3]));

		dvec_copy(feet[side].w_mid_vel+3, feet[side].w_sensor_vel+3, 3);
		dvec_copy(feet[side].w_toe_vel+3, feet[side].w_sensor_vel+3, 3);
		dvec_copy(feet[side].w_heel_vel+3, feet[side].w_sensor_vel+3, 3);

		//hand angular velocities
		double tmpW[3];
		model->sdangvel(handBodyInd[side], tmpW);
		model->sdtrans(handBodyInd[side], tmpW, -1, &(handd[side][3]));

		// foot tip
		for (int i = 0; i < 4; i++) {
			dvec_add(com_to_ankle, Foot::ankle_to_tip_offset[i], temp, 3);

			model->sdpos(body[side], temp, feet[side].vertices[i]);
		}
	}

	model->sdorient(PELV_S_BODY_PELVIS, dircos);
	model->sddc2ang(dircos, root+3, root+4, root+5);

	// fill out various body part locations
	double offset[3];
	for (int i = 1; i < N_BODIES; i++) {
		model->sdgetbtj(i, offset);
		model->sdpos(i, offset, &(locations[i-1][0]));
	}

	//mass, com position, and total moment of inertia
	model->sdsys(&m, com, (double (*)[3])I.data());

	//comd from momentum
	for(int i = 0; i < 3; i++)
		comd[i] = mom[i]/m;		//compute comd from momentum

	// pelvis angular velocity in world frame
	model->sdangvel(PELV_S_BODY_PELVIS, temp);
	model->sdtrans(PELV_S_BODY_PELVIS, temp, -1, rootd+3);

	// utorso orientation
	body_position(model, NULL, PELV_S_BODY_UP_TORSO, zeros, temp, Jtorso.data());
	model->sdorient(PELV_S_BODY_UP_TORSO, dircos);
	model->sddc2quat(dircos, qq, qq+1, qq+2, qq+3);
	utorsoq = Eigen::Quaterniond(qq[3], qq[0], qq[1], qq[2]).normalized();
	model->sdangvel(PELV_S_BODY_UP_TORSO, temp);
	model->sdtrans(PELV_S_BODY_UP_TORSO, temp, -1, utorsow);

	// pelvis orientation
	body_position(model, NULL, PELV_S_BODY_PELVIS, zeros, temp, Jpelvis.data());
	model->sdorient(PELV_S_BODY_PELVIS, dircos);
	model->sddc2quat(dircos, qq, qq+1, qq+2, qq+3);
	pelvisq = Eigen::Quaterniond(qq[3], qq[0], qq[1], qq[2]).normalized();
	model->sdangvel(PELV_S_BODY_PELVIS, temp);
	model->sdtrans(PELV_S_BODY_PELVIS, temp, -1, pelvisw);

	//mass matrix
	model->sdmassmat(M.data());

	//com jacobian
	com_jacobian(model, Jc.data());

	//com inertia
	model->sdgetiner(PELV_S_BODY_PELVIS, (double (*)[3])Itorso.data());

	// non lin part
	model->sdfrcmat(nonLin.data());

	// compute Jd and Jcd by integrating one step to get next J
	///////////////////////////////////////////////////////////////////////////////////
	Eigen::Matrix<double,PELV_N_SDFAST_U,6,Eigen::RowMajor> J2[LR];
	Eigen::Matrix<double,PELV_N_SDFAST_U,6,Eigen::RowMajor> Jhand2[LR];
	Eigen::Matrix<double,PELV_N_SDFAST_U,3,Eigen::RowMajor> Jc2;
	Eigen::Matrix<double,PELV_N_SDFAST_U,6,Eigen::RowMajor> Jtorso2;
	Eigen::Matrix<double,PELV_N_SDFAST_U,6,Eigen::RowMajor> Jpelvis2;

	double qdot[PELV_N_SDFAST_Q];
	model->sdu2qdot(&_sdFastState[PELV_N_SDFAST_Q], qdot);
	// set new state
	for (int i = 0; i < PELV_N_SDFAST_Q; i++)
		_sdFastState[i] += qdot[i] * timeStep;
	set_state(model, _sdFastState);

	// foot positions and jacobians
	for (int side = 0; side < 2; side++) {
		model->sdgetbtj(joint[side], com_to_ankle);
		// ref point
		dvec_add(com_to_ankle, Foot::ankle_to_sensor_offset, temp, 3);
		body_position(model, NULL, body[side], temp, NULL, J2[side].data());
		Jd[side] = (J2[side] - J[side]) / timeStep;

		//mid of foot
		dvec_add(com_to_ankle, feet[side].ankle_to_center_offset, temp, 3);
		body_position(model, NULL, body[side], temp, NULL, J2[side].data());
		Jmidd[side] = (J2[side] - Jmid[side]) / timeStep;

		//toe
		dvec_add(com_to_ankle, feet[side].ankle_to_toe_offset, temp, 3);
		body_position(model, NULL, body[side], temp, NULL, J2[side].data());
		Jtoed[side] = (J2[side] - Jtoe[side]) / timeStep;

		//heel
		dvec_add(com_to_ankle, feet[side].ankle_to_heel_offset, temp, 3);
		body_position(model, NULL, body[side], temp, NULL, J2[side].data());
		Jheeld[side] = (J2[side] - Jheel[side]) / timeStep;

		double handOffset[3];
		model->sdgetbtj(handBodyInd[side], handOffset);
		for(int i = 0; i < 3; i++)	handOffset[i] += wrist2hand[side][i];
		body_position(model, NULL, handBodyInd[side], handOffset, NULL, Jhand2[side].data());
		Jhandd[side] = (Jhand2[side] - Jhand[side]) / timeStep;
	}
	// com
	com_jacobian(model, Jc2.data());
	Jcd = (Jc2 - Jc) / timeStep;

	// utorso
	body_position(model, NULL, PELV_S_BODY_UP_TORSO, zeros, temp, Jtorso2.data());
	Jtorsod = (Jtorso2 - Jtorso) / timeStep;

	//pelvis
	body_position(model, NULL, PELV_S_BODY_PELVIS, zeros, temp, Jpelvis2.data());
	Jpelvisd = (Jpelvis2 - Jpelvis) / timeStep;

	// restore
	for (int i = 0; i < PELV_N_SDFAST_Q; i++)
		_sdFastState[i] -= qdot[i] * timeStep;
	set_state(model, _sdFastState);
	///////////////////////////////////////////////////////////////////////////////////

	model->sdprinterr(stderr);
	//sdclearerr();
	checkContact();

#ifdef ATLAS_ONLINE
	//empirical CoM offset for robot
	double handOffset[3];
	for(int i = 0; i < 2; i++) {
		handOffset[i] = (hand[LEFT][i] + hand[RIGHT][i])*0.5;
		com[i] += handOffset[i]/0.8*0.03;
	}

#endif

	for(int i = 0; i < 3; i++)	com[i] += comOffset[i];
}

void PelvRobotState::makeSDFastState(const double pos[3], const Eigen::Quaterniond &q, 
		const double vel[3], const double w[3],
		const double j[N_JOINTS], const double jd[N_JOINTS],
		double *sdfast_state)
{
	for (int i = 0; i < PELV_N_SDFAST_STATE; i++)
		sdfast_state[i] = 0;

	// copy joint pos and vel
	memcpy(sdfast_state+6, j, sizeof(double)*N_JOINTS);
	memcpy(sdfast_state+PELV_N_SDFAST_Q+6, jd, sizeof(double)*N_JOINTS);

	// get position and vel
	memcpy(sdfast_state, pos, sizeof(double)*XYZ);
	sdfast_state[PELV_S_Q0] = q.x();
	sdfast_state[PELV_S_Q1] = q.y();
	sdfast_state[PELV_S_Q2] = q.z();
	sdfast_state[PELV_S_Q3] = q.w();

	memcpy(sdfast_state+PELV_N_SDFAST_Q, vel, sizeof(double)*XYZ);
	memcpy(sdfast_state+PELV_N_SDFAST_Q+3, w, sizeof(double)*XYZ);
}

void PelvRobotState::addToLog(BatchLogger &logger) {
	logger.add_datapoint("RS.cdt","s",&ctrl_dt);

	logger.add_datapoint("RS.cState", "-", &contactState);
	logger.add_datapoint("RS.com[X]","m",&(com[XX]));
	logger.add_datapoint("RS.com[Y]","m",&(com[YY]));
	logger.add_datapoint("RS.com[Z]","m",&(com[ZZ]));

	logger.add_datapoint("RS.comd[X]","v",&(comd[XX]));
	logger.add_datapoint("RS.comd[Y]","v",&(comd[YY]));
	logger.add_datapoint("RS.comd[Z]","v",&(comd[ZZ]));

	logger.add_datapoint("RS.cop[X]","a",&(cop[XX]));
	logger.add_datapoint("RS.cop[Y]","a",&(cop[YY]));
	logger.add_datapoint("RS.foot[L].b_cop[X]","m",&(feet[LEFT].b_cop[XX]));
	logger.add_datapoint("RS.foot[L].b_cop[Y]","m",&(feet[LEFT].b_cop[YY]));
	logger.add_datapoint("RS.foot[R].b_cop[X]","m",&(feet[RIGHT].b_cop[XX]));
	logger.add_datapoint("RS.foot[R].b_cop[Y]","m",&(feet[RIGHT].b_cop[YY]));

	logger.add_quat("RS.pelvQ",&(pelvisq));
	logger.add_quat("RS.torsoQ",&(utorsoq));

	logger.add_datapoint("RS.foot[L][X]","m",&(feet[LEFT].w_sensor_pos[XX]));
	logger.add_datapoint("RS.foot[L][Y]","m",&(feet[LEFT].w_sensor_pos[YY]));
	logger.add_datapoint("RS.foot[L][Z]","m",&(feet[LEFT].w_sensor_pos[ZZ]));
	logger.add_datapoint("RS.foot[R][X]","m",&(feet[RIGHT].w_sensor_pos[XX]));
	logger.add_datapoint("RS.foot[R][Y]","m",&(feet[RIGHT].w_sensor_pos[YY]));
	logger.add_datapoint("RS.foot[R][Z]","m",&(feet[RIGHT].w_sensor_pos[ZZ]));

	logger.add_quat("RS.footQ[L]",&(feet[LEFT].w_q));
	logger.add_quat("RS.footQ[R]",&(feet[RIGHT].w_q));

	logger.add_datapoint("RS.footd[L][X]","v",&(feet[LEFT].w_sensor_vel[XX]));
	logger.add_datapoint("RS.footd[L][Y]","v",&(feet[LEFT].w_sensor_vel[YY]));
	logger.add_datapoint("RS.footd[L][Z]","v",&(feet[LEFT].w_sensor_vel[ZZ]));
	logger.add_datapoint("RS.footd[R][X]","v",&(feet[RIGHT].w_sensor_vel[XX]));
	logger.add_datapoint("RS.footd[R][Y]","v",&(feet[RIGHT].w_sensor_vel[YY]));
	logger.add_datapoint("RS.footd[R][Z]","v",&(feet[RIGHT].w_sensor_vel[ZZ]));

	logger.add_datapoint("RS.hand[L][X]","m",&(hand[LEFT][XX]));
	logger.add_datapoint("RS.hand[L][Y]","m",&(hand[LEFT][YY]));
	logger.add_datapoint("RS.hand[L][Z]","m",&(hand[LEFT][ZZ]));
	logger.add_datapoint("RS.hand[R][X]","m",&(hand[RIGHT][XX]));
	logger.add_datapoint("RS.hand[R][Y]","m",&(hand[RIGHT][YY]));
	logger.add_datapoint("RS.hand[R][Z]","m",&(hand[RIGHT][ZZ]));

	logger.add_quat("RS.handQ[L]",&(handQ[LEFT]));
	logger.add_quat("RS.handQ[R]",&(handQ[RIGHT]));

	logger.add_datapoint("RS.hAxis[L][X]","m",&(handAxis[LEFT][XX]));
	logger.add_datapoint("RS.hAxis[L][Y]","m",&(handAxis[LEFT][YY]));
	logger.add_datapoint("RS.hAxis[L][Z]","m",&(handAxis[LEFT][ZZ]));
	logger.add_datapoint("RS.hAxis[R][X]","m",&(handAxis[RIGHT][XX]));
	logger.add_datapoint("RS.hAxis[R][Y]","m",&(handAxis[RIGHT][YY]));
	logger.add_datapoint("RS.hAxis[R][Z]","m",&(handAxis[RIGHT][ZZ]));

	logger.add_datapoint("RS.dAxis[L][X]","m",&(drillAxis[LEFT][XX]));
	logger.add_datapoint("RS.dAxis[L][Y]","m",&(drillAxis[LEFT][YY]));
	logger.add_datapoint("RS.dAxis[L][Z]","m",&(drillAxis[LEFT][ZZ]));
	logger.add_datapoint("RS.dAxis[R][X]","m",&(drillAxis[RIGHT][XX]));
	logger.add_datapoint("RS.dAxis[R][Y]","m",&(drillAxis[RIGHT][YY]));
	logger.add_datapoint("RS.dAxis[R][Z]","m",&(drillAxis[RIGHT][ZZ]));

	logger.add_datapoint("RS.elbow[L][X]","m",&(elbow[LEFT][XX]));
	logger.add_datapoint("RS.elbow[L][Y]","m",&(elbow[LEFT][YY]));
	logger.add_datapoint("RS.elbow[L][Z]","m",&(elbow[LEFT][ZZ]));
	logger.add_datapoint("RS.elbow[R][X]","m",&(elbow[RIGHT][XX]));
	logger.add_datapoint("RS.elbow[R][Y]","m",&(elbow[RIGHT][YY]));
	logger.add_datapoint("RS.elbow[R][Z]","m",&(elbow[RIGHT][ZZ]));

	logger.add_datapoint("RS.pelvW[X]","r/s",&(pelvisw[XX]));
	logger.add_datapoint("RS.pelvW[Y]","r/s",&(pelvisw[YY]));
	logger.add_datapoint("RS.pelvW[Z]","r/s",&(pelvisw[ZZ]));

	logger.add_datapoint("RS.torsoW[X]","r/s",&(utorsow[XX]));
	logger.add_datapoint("RS.torsoW[Y]","r/s",&(utorsow[YY]));
	logger.add_datapoint("RS.torsoW[Z]","r/s",&(utorsow[ZZ]));

	logger.add_datapoint("RS.F[L][Z]","N",&(feet[LEFT].w_F[ZZ]));
	logger.add_datapoint("RS.F[R][Z]","N",&(feet[RIGHT].w_F[ZZ]));

	logger.add_datapoint("RS.M[L][X]","Nm",&(feet[LEFT].w_F[XX+3]));
	logger.add_datapoint("RS.M[L][Y]","Nm",&(feet[LEFT].w_F[YY+3]));
	logger.add_datapoint("RS.M[R][X]","Nm",&(feet[RIGHT].w_F[XX+3]));
	logger.add_datapoint("RS.M[R][Y]","Nm",&(feet[RIGHT].w_F[YY+3]));

	logger.add_datapoint("RS.HF[L][X]","N",&(handForces[LEFT][XX]));
	logger.add_datapoint("RS.HF[L][Y]","N",&(handForces[LEFT][YY]));
	logger.add_datapoint("RS.HF[L][Z]","N",&(handForces[LEFT][ZZ]));
	logger.add_datapoint("RS.HF[R][X]","N",&(handForces[RIGHT][XX]));
	logger.add_datapoint("RS.HF[R][Y]","N",&(handForces[RIGHT][YY]));
	logger.add_datapoint("RS.HF[R][Z]","N",&(handForces[RIGHT][ZZ]));
	logger.add_datapoint("RS.HF[L][X]","Nm",&(handForces[LEFT][XX+3]));
	logger.add_datapoint("RS.HM[L][Y]","Nm",&(handForces[LEFT][YY+3]));
	logger.add_datapoint("RS.HM[L][Z]","Nm",&(handForces[LEFT][ZZ+3]));
	logger.add_datapoint("RS.HM[R][X]","Nm",&(handForces[RIGHT][XX+3]));
	logger.add_datapoint("RS.HM[R][Y]","Nm",&(handForces[RIGHT][YY+3]));
	logger.add_datapoint("RS.HM[R][Z]","Nm",&(handForces[RIGHT][ZZ+3]));

	logger.add_datapoint("RS.ftContact[L]","-",&(footContact[LEFT]));
	logger.add_datapoint("RS.ftContact[R]","-",&(footContact[RIGHT]));

	logger.add_datapoint("RS.root[X]","m",&(root[XX]));
	logger.add_datapoint("RS.root[Y]","m",&(root[YY]));
	logger.add_datapoint("RS.root[Z]","m",&(root[ZZ]));
	logger.add_datapoint("RS.rootd[X]","m",&(rootd[XX]));
	logger.add_datapoint("RS.rootd[Y]","m",&(rootd[YY]));
	logger.add_datapoint("RS.rootd[Z]","m",&(rootd[ZZ]));

	logger.add_datapoint("RS.joint[back_lbz]","r",&(joints[0]));
	logger.add_datapoint("RS.joint[back_mby]","r",&(joints[1]));
	logger.add_datapoint("RS.joint[back_ubx]","r",&(joints[2]));
	logger.add_datapoint("RS.joint[neck_ay]","r",&(joints[3]));
	logger.add_datapoint("RS.joint[l_leg_uhz]","r",&(joints[4]));
	logger.add_datapoint("RS.joint[l_leg_mhx]","r",&(joints[5]));
	logger.add_datapoint("RS.joint[l_leg_lhy]","r",&(joints[6]));
	logger.add_datapoint("RS.joint[l_leg_kny]","r",&(joints[7]));
	logger.add_datapoint("RS.joint[l_leg_uay]","r",&(joints[8]));
	logger.add_datapoint("RS.joint[l_leg_lax]","r",&(joints[9]));
	logger.add_datapoint("RS.joint[r_leg_uhz]","r",&(joints[10]));
	logger.add_datapoint("RS.joint[r_leg_mhx]","r",&(joints[11]));
	logger.add_datapoint("RS.joint[r_leg_lhy]","r",&(joints[12]));
	logger.add_datapoint("RS.joint[r_leg_kny]","r",&(joints[13]));
	logger.add_datapoint("RS.joint[r_leg_uay]","r",&(joints[14]));
	logger.add_datapoint("RS.joint[r_leg_lax]","r",&(joints[15]));
	logger.add_datapoint("RS.joint[l_arm_usy]","r",&(joints[16]));
	logger.add_datapoint("RS.joint[l_arm_shx]","r",&(joints[17]));
	logger.add_datapoint("RS.joint[l_arm_ely]","r",&(joints[18]));
	logger.add_datapoint("RS.joint[l_arm_elx]","r",&(joints[19]));
	logger.add_datapoint("RS.joint[l_arm_uwy]","r",&(joints[20]));
	logger.add_datapoint("RS.joint[l_arm_mwx]","r",&(joints[21]));
	logger.add_datapoint("RS.joint[r_arm_usy]","r",&(joints[22]));
	logger.add_datapoint("RS.joint[r_arm_shx]","r",&(joints[23]));
	logger.add_datapoint("RS.joint[r_arm_ely]","r",&(joints[24]));
	logger.add_datapoint("RS.joint[r_arm_elx]","r",&(joints[25]));
	logger.add_datapoint("RS.joint[r_arm_uwy]","r",&(joints[26]));
	logger.add_datapoint("RS.joint[r_arm_mwx]","r",&(joints[27]));

	logger.add_datapoint("RS.jointd[back_lbz]","r/s",&(jointsd[0]));
	logger.add_datapoint("RS.jointd[back_mby]","r/s",&(jointsd[1]));
	logger.add_datapoint("RS.jointd[back_ubx]","r/s",&(jointsd[2]));
	logger.add_datapoint("RS.jointd[neck_ay]","r/s",&(jointsd[3]));
	logger.add_datapoint("RS.jointd[l_leg_uhz]","r/s",&(jointsd[4]));
	logger.add_datapoint("RS.jointd[l_leg_mhx]","r/s",&(jointsd[5]));
	logger.add_datapoint("RS.jointd[l_leg_lhy]","r/s",&(jointsd[6]));
	logger.add_datapoint("RS.jointd[l_leg_kny]","r/s",&(jointsd[7]));
	logger.add_datapoint("RS.jointd[l_leg_uay]","r/s",&(jointsd[8]));
	logger.add_datapoint("RS.jointd[l_leg_lax]","r/s",&(jointsd[9]));
	logger.add_datapoint("RS.jointd[r_leg_uhz]","r/s",&(jointsd[10]));
	logger.add_datapoint("RS.jointd[r_leg_mhx]","r/s",&(jointsd[11]));
	logger.add_datapoint("RS.jointd[r_leg_lhy]","r/s",&(jointsd[12]));
	logger.add_datapoint("RS.jointd[r_leg_kny]","r/s",&(jointsd[13]));
	logger.add_datapoint("RS.jointd[r_leg_uay]","r/s",&(jointsd[14]));
	logger.add_datapoint("RS.jointd[r_leg_lax]","r/s",&(jointsd[15]));
	logger.add_datapoint("RS.jointd[l_arm_usy]","r/s",&(jointsd[16]));
	logger.add_datapoint("RS.jointd[l_arm_shx]","r/s",&(jointsd[17]));
	logger.add_datapoint("RS.jointd[l_arm_ely]","r/s",&(jointsd[18]));
	logger.add_datapoint("RS.jointd[l_arm_elx]","r/s",&(jointsd[19]));
	logger.add_datapoint("RS.jointd[l_arm_uwy]","r/s",&(jointsd[20]));
	logger.add_datapoint("RS.jointd[l_arm_mwx]","r/s",&(jointsd[21]));
	logger.add_datapoint("RS.jointd[r_arm_usy]","r/s",&(jointsd[22]));
	logger.add_datapoint("RS.jointd[r_arm_shx]","r/s",&(jointsd[23]));
	logger.add_datapoint("RS.jointd[r_arm_ely]","r/s",&(jointsd[24]));
	logger.add_datapoint("RS.jointd[r_arm_elx]","r/s",&(jointsd[25]));
	logger.add_datapoint("RS.jointd[r_arm_uwy]","r/s",&(jointsd[26]));
	logger.add_datapoint("RS.jointd[r_arm_mwx]","r/s",&(jointsd[27]));

	logger.add_datapoint("RS.trq[back_lbz]","Nm",&(joints_torque[0]));
	logger.add_datapoint("RS.trq[back_mby]","Nm",&(joints_torque[1]));
	logger.add_datapoint("RS.trq[back_ubx]","Nm",&(joints_torque[2]));
	logger.add_datapoint("RS.trq[neck_ay]","Nm",&(joints_torque[3]));
	logger.add_datapoint("RS.trq[l_leg_uhz]","Nm",&(joints_torque[4]));
	logger.add_datapoint("RS.trq[l_leg_mhx]","Nm",&(joints_torque[5]));
	logger.add_datapoint("RS.trq[l_leg_lhy]","Nm",&(joints_torque[6]));
	logger.add_datapoint("RS.trq[l_leg_kny]","Nm",&(joints_torque[7]));
	logger.add_datapoint("RS.trq[l_leg_uay]","Nm",&(joints_torque[8]));
	logger.add_datapoint("RS.trq[l_leg_lax]","Nm",&(joints_torque[9]));
	logger.add_datapoint("RS.trq[r_leg_uhz]","Nm",&(joints_torque[10]));
	logger.add_datapoint("RS.trq[r_leg_mhx]","Nm",&(joints_torque[11]));
	logger.add_datapoint("RS.trq[r_leg_lhy]","Nm",&(joints_torque[12]));
	logger.add_datapoint("RS.trq[r_leg_kny]","Nm",&(joints_torque[13]));
	logger.add_datapoint("RS.trq[r_leg_uay]","Nm",&(joints_torque[14]));
	logger.add_datapoint("RS.trq[r_leg_lax]","Nm",&(joints_torque[15]));
	logger.add_datapoint("RS.trq[l_arm_usy]","Nm",&(joints_torque[16]));
	logger.add_datapoint("RS.trq[l_arm_shx]","Nm",&(joints_torque[17]));
	logger.add_datapoint("RS.trq[l_arm_ely]","Nm",&(joints_torque[18]));
	logger.add_datapoint("RS.trq[l_arm_elx]","Nm",&(joints_torque[19]));
	logger.add_datapoint("RS.trq[l_arm_uwy]","Nm",&(joints_torque[20]));
	logger.add_datapoint("RS.trq[l_arm_mwx]","Nm",&(joints_torque[21]));
	logger.add_datapoint("RS.trq[r_arm_usy]","Nm",&(joints_torque[22]));
	logger.add_datapoint("RS.trq[r_arm_shx]","Nm",&(joints_torque[23]));
	logger.add_datapoint("RS.trq[r_arm_ely]","Nm",&(joints_torque[24]));
	logger.add_datapoint("RS.trq[r_arm_elx]","Nm",&(joints_torque[25]));
	logger.add_datapoint("RS.trq[r_arm_uwy]","Nm",&(joints_torque[26]));
	logger.add_datapoint("RS.trq[r_arm_mwx]","Nm",&(joints_torque[27]));

}
