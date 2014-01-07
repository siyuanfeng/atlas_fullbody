//Code by Eric Whitman
//Based on code by Siyuan Feng
//Based on code by Eric Whitman
//Based on code by Ben Stephens


#include "ik_controller.hpp"
#include "drc_pelvis_defines.h"
#include "eiquadprog.hpp"
#include "Utils.hpp"
#include "Eigen_utils.hpp"
#include "Logger.h"

double refJoints[PELV_N_JOINTS] = {0, 0, 0, 0,
		0, 0.06,-0.23, 0.52, -0.28, -0.06, //left leg
		0, -0.06,-0.23, 0.52, -0.28, 0.06, //right leg
		0.3, -1.3, 2.0, 0.5, 0, 0, // left arm
		0.3,  1.3, 2.0, -0.5, 0, 0 // right arm
};

bool PelvIkCon::setToRobotState(const RobotState &rs) {
	if (rs.getType() != RobotState::TYPE_PELVIS)
		assert(0);

	ikrs->timeStep = rs.timeStep;
	ikrs->computeSDFvars(rs.root, rs.rootq.coeffs().data(), rs.rootd, rs.root_b_w, rs.joints, rs.jointsd);
	return true;
}

void PelvIkCon::setPose(const double *theta, const double *thetad) {
	for(int i = 0;i < PELV_N_JOINTS; i++) {
		ikrs->joints[i] = theta[i];
		ikrs->jointsd[i] = thetad[i];
	}
	ikrs->computeSDFvars();
}

void PelvIkCon::matchRoot(const RobotState &rs) {
	setRoot(rs.root, rs.rootq, rs.rootd, rs.root_b_w);
}

void PelvIkCon::setRoot(const double *root, Eigen::Quaterniond rootQ, const double *rootd, const double *rootW) {
	for(int i = 0; i < 3; i++)	ikrs->root[i] = root[i];
	ikrs->rootq = rootQ;
	if(rootd)		for(int i = 0; i < 3; i++)	ikrs->rootd[i] = rootd[i];
	if(rootW)		for(int i = 0; i < 3; i++)	ikrs->root_b_w[i] = rootW[i];
	ikrs->computeSDFvars();
}

void PelvIkCon::setToRefPose() {
	for(int i = 0;i < PELV_N_JOINTS; i++) {
		ikrs->joints[i] = refJoints[i];
		ikrs->jointsd[i] = 0.0;
	}
	ikrs->computeSDFvars();
}

void PelvIkCon::addToLog(BatchLogger &logger) {
	logger.add_datapoint("IK.outOfLimits","-",&outOfLimits);
	logger.add_datapoint("IK.QPval","-",&QPval);
	logger.add_datapoint("IK.computeCount","-",&computeCount);
	logger.add_datapoint("IK.com[X]","m",&(ikrs->com[XX]));
	logger.add_datapoint("IK.com[Y]","m",&(ikrs->com[YY]));
	logger.add_datapoint("IK.com[Z]","m",&(ikrs->com[ZZ]));

	logger.add_datapoint("IK.comd[X]","v",&(ikrs->comd[XX]));
	logger.add_datapoint("IK.comd[Y]","v",&(ikrs->comd[YY]));
	logger.add_datapoint("IK.comd[Z]","v",&(ikrs->comd[ZZ]));

	logger.add_quat("IK.pelvQ",&(ikrs->pelvisq));
	logger.add_quat("IK.torsoQ",&(ikrs->utorsoq));

	logger.add_datapoint("IK.foot[L][X]","m",&(ikrs->feet[LEFT].w_sensor_pos[XX]));
	logger.add_datapoint("IK.foot[L][Y]","m",&(ikrs->feet[LEFT].w_sensor_pos[YY]));
	logger.add_datapoint("IK.foot[L][Z]","m",&(ikrs->feet[LEFT].w_sensor_pos[ZZ]));
	logger.add_datapoint("IK.foot[R][X]","m",&(ikrs->feet[RIGHT].w_sensor_pos[XX]));
	logger.add_datapoint("IK.foot[R][Y]","m",&(ikrs->feet[RIGHT].w_sensor_pos[YY]));
	logger.add_datapoint("IK.foot[R][Z]","m",&(ikrs->feet[RIGHT].w_sensor_pos[ZZ]));

	logger.add_quat("IK.footQ[L]",&(ikrs->feet[LEFT].w_q));
	logger.add_quat("IK.footQ[R]",&(ikrs->feet[RIGHT].w_q));

	logger.add_datapoint("IK.hand[L][X]","m",&(ikrs->hand[LEFT][XX]));
	logger.add_datapoint("IK.hand[L][Y]","m",&(ikrs->hand[LEFT][YY]));
	logger.add_datapoint("IK.hand[L][Z]","m",&(ikrs->hand[LEFT][ZZ]));
	logger.add_datapoint("IK.hand[R][X]","m",&(ikrs->hand[RIGHT][XX]));
	logger.add_datapoint("IK.hand[R][Y]","m",&(ikrs->hand[RIGHT][YY]));
	logger.add_datapoint("IK.hand[R][Z]","m",&(ikrs->hand[RIGHT][ZZ]));

	logger.add_quat("IK.handQ[L]",&(ikrs->handQ[LEFT]));
	logger.add_quat("IK.handQ[R]",&(ikrs->handQ[RIGHT]));

	logger.add_datapoint("IK.hAxis[L][X]","m",&(ikrs->handAxis[LEFT][XX]));
	logger.add_datapoint("IK.hAxis[L][Y]","m",&(ikrs->handAxis[LEFT][YY]));
	logger.add_datapoint("IK.hAxis[L][Z]","m",&(ikrs->handAxis[LEFT][ZZ]));
	logger.add_datapoint("IK.hAxis[R][X]","m",&(ikrs->handAxis[RIGHT][XX]));
	logger.add_datapoint("IK.hAxis[R][Y]","m",&(ikrs->handAxis[RIGHT][YY]));
	logger.add_datapoint("IK.hAxis[R][Z]","m",&(ikrs->handAxis[RIGHT][ZZ]));

	logger.add_datapoint("IK.elbow[L][X]","m",&(ikrs->elbow[LEFT][XX]));
	logger.add_datapoint("IK.elbow[L][Y]","m",&(ikrs->elbow[LEFT][YY]));
	logger.add_datapoint("IK.elbow[L][Z]","m",&(ikrs->elbow[LEFT][ZZ]));
	logger.add_datapoint("IK.elbow[R][X]","m",&(ikrs->elbow[RIGHT][XX]));
	logger.add_datapoint("IK.elbow[R][Y]","m",&(ikrs->elbow[RIGHT][YY]));
	logger.add_datapoint("IK.elbow[R][Z]","m",&(ikrs->elbow[RIGHT][ZZ]));

	logger.add_datapoint("IK.pelvW[X]","r/s",&(ikrs->pelvisw[XX]));
	logger.add_datapoint("IK.pelvW[Y]","r/s",&(ikrs->pelvisw[YY]));
	logger.add_datapoint("IK.pelvW[Z]","r/s",&(ikrs->pelvisw[ZZ]));

	logger.add_datapoint("IK.torsoW[X]","r/s",&(ikrs->utorsow[XX]));
	logger.add_datapoint("IK.torsoW[Y]","r/s",&(ikrs->utorsow[YY]));
	logger.add_datapoint("IK.torsoW[Z]","r/s",&(ikrs->utorsow[ZZ]));

	logger.add_datapoint("IK.root[X]","m",&(ikrs->root[XX]));
	logger.add_datapoint("IK.root[Y]","m",&(ikrs->root[YY]));
	logger.add_datapoint("IK.root[Z]","m",&(ikrs->root[ZZ]));
	logger.add_datapoint("IK.rootd[X]","v",&(ikrs->rootd[XX]));
	logger.add_datapoint("IK.rootd[Y]","v",&(ikrs->rootd[YY]));
	logger.add_datapoint("IK.rootd[Z]","v",&(ikrs->rootd[ZZ]));
	logger.add_datapoint("IK.quat[X]","-",&(ikrs->rootq.coeffs().data()[0]));
	logger.add_datapoint("IK.quat[Y]","-",&(ikrs->rootq.coeffs().data()[1]));
	logger.add_datapoint("IK.quat[Z]","-",&(ikrs->rootq.coeffs().data()[2]));
	logger.add_datapoint("IK.quat[W]","-",&(ikrs->rootq.coeffs().data()[3]));
	logger.add_datapoint("IK.jt[back_lbz]","-",&(ikrs->joints[0]));
	logger.add_datapoint("IK.jt[back_mby]","-",&(ikrs->joints[1]));
	logger.add_datapoint("IK.jt[back_ubx]","-",&(ikrs->joints[2]));
	logger.add_datapoint("IK.jt[neck_ay]","-",&(ikrs->joints[3]));
	logger.add_datapoint("IK.jt[l_leg_uhz]","-",&(ikrs->joints[4]));
	logger.add_datapoint("IK.jt[l_leg_mhx]","-",&(ikrs->joints[5]));
	logger.add_datapoint("IK.jt[l_leg_lhy]","-",&(ikrs->joints[6]));
	logger.add_datapoint("IK.jt[l_leg_kny]","-",&(ikrs->joints[7]));
	logger.add_datapoint("IK.jt[l_leg_uay]","-",&(ikrs->joints[8]));
	logger.add_datapoint("IK.jt[l_leg_lax]","-",&(ikrs->joints[9]));
	logger.add_datapoint("IK.jt[r_leg_uhz]","-",&(ikrs->joints[10]));
	logger.add_datapoint("IK.jt[r_leg_mhx]","-",&(ikrs->joints[11]));
	logger.add_datapoint("IK.jt[r_leg_lhy]","-",&(ikrs->joints[12]));
	logger.add_datapoint("IK.jt[r_leg_kny]","-",&(ikrs->joints[13]));
	logger.add_datapoint("IK.jt[r_leg_uay]","-",&(ikrs->joints[14]));
	logger.add_datapoint("IK.jt[r_leg_lax]","-",&(ikrs->joints[15]));
	logger.add_datapoint("IK.jt[l_arm_usy]","-",&(ikrs->joints[16]));
	logger.add_datapoint("IK.jt[l_arm_shx]","-",&(ikrs->joints[17]));
	logger.add_datapoint("IK.jt[l_arm_ely]","-",&(ikrs->joints[18]));
	logger.add_datapoint("IK.jt[l_arm_elx]","-",&(ikrs->joints[19]));
	logger.add_datapoint("IK.jt[l_arm_uwy]","-",&(ikrs->joints[20]));
	logger.add_datapoint("IK.jt[l_arm_mwx]","-",&(ikrs->joints[21]));
	logger.add_datapoint("IK.jt[r_arm_usy]","-",&(ikrs->joints[22]));
	logger.add_datapoint("IK.jt[r_arm_shx]","-",&(ikrs->joints[23]));
	logger.add_datapoint("IK.jt[r_arm_ely]","-",&(ikrs->joints[24]));
	logger.add_datapoint("IK.jt[r_arm_elx]","-",&(ikrs->joints[25]));
	logger.add_datapoint("IK.jt[r_arm_uwy]","-",&(ikrs->joints[26]));
	logger.add_datapoint("IK.jt[r_arm_mwx]","-",&(ikrs->joints[27]));

	logger.add_datapoint("IK.jtd[back_lbz]","-",&(ikrs->jointsd[0]));
	logger.add_datapoint("IK.jtd[back_mby]","-",&(ikrs->jointsd[1]));
	logger.add_datapoint("IK.jtd[back_ubx]","-",&(ikrs->jointsd[2]));
	logger.add_datapoint("IK.jtd[neck_ay]","-",&(ikrs->jointsd[3]));
	logger.add_datapoint("IK.jtd[l_leg_uhz]","-",&(ikrs->jointsd[4]));
	logger.add_datapoint("IK.jtd[l_leg_mhx]","-",&(ikrs->jointsd[5]));
	logger.add_datapoint("IK.jtd[l_leg_lhy]","-",&(ikrs->jointsd[6]));
	logger.add_datapoint("IK.jtd[l_leg_kny]","-",&(ikrs->jointsd[7]));
	logger.add_datapoint("IK.jtd[l_leg_uay]","-",&(ikrs->jointsd[8]));
	logger.add_datapoint("IK.jtd[l_leg_lax]","-",&(ikrs->jointsd[9]));
	logger.add_datapoint("IK.jtd[r_leg_uhz]","-",&(ikrs->jointsd[10]));
	logger.add_datapoint("IK.jtd[r_leg_mhx]","-",&(ikrs->jointsd[11]));
	logger.add_datapoint("IK.jtd[r_leg_lhy]","-",&(ikrs->jointsd[12]));
	logger.add_datapoint("IK.jtd[r_leg_kny]","-",&(ikrs->jointsd[13]));
	logger.add_datapoint("IK.jtd[r_leg_uay]","-",&(ikrs->jointsd[14]));
	logger.add_datapoint("IK.jtd[r_leg_lax]","-",&(ikrs->jointsd[15]));
	logger.add_datapoint("IK.jtd[l_arm_usy]","-",&(ikrs->jointsd[16]));
	logger.add_datapoint("IK.jtd[l_arm_shx]","-",&(ikrs->jointsd[17]));
	logger.add_datapoint("IK.jtd[l_arm_ely]","-",&(ikrs->jointsd[18]));
	logger.add_datapoint("IK.jtd[l_arm_elx]","-",&(ikrs->jointsd[19]));
	logger.add_datapoint("IK.jtd[l_arm_uwy]","-",&(ikrs->jointsd[20]));
	logger.add_datapoint("IK.jtd[l_arm_mwx]","-",&(ikrs->jointsd[21]));
	logger.add_datapoint("IK.jtd[r_arm_usy]","-",&(ikrs->jointsd[22]));
	logger.add_datapoint("IK.jtd[r_arm_shx]","-",&(ikrs->jointsd[23]));
	logger.add_datapoint("IK.jtd[r_arm_ely]","-",&(ikrs->jointsd[24]));
	logger.add_datapoint("IK.jtd[r_arm_elx]","-",&(ikrs->jointsd[25]));
	logger.add_datapoint("IK.jtd[r_arm_uwy]","-",&(ikrs->jointsd[26]));
	logger.add_datapoint("IK.jtd[r_arm_mwx]","-",&(ikrs->jointsd[27]));

	logger.add_datapoint("IK.fixed[back_lbz]","-",&(forceMask[0]));
	logger.add_datapoint("IK.fixed[back_mby]","-",&(forceMask[1]));
	logger.add_datapoint("IK.fixed[back_ubx]","-",&(forceMask[2]));
	logger.add_datapoint("IK.fixed[neck_ay]","-",&(forceMask[3]));
	logger.add_datapoint("IK.fixed[l_leg_uhz]","-",&(forceMask[4]));
	logger.add_datapoint("IK.fixed[l_leg_mhx]","-",&(forceMask[5]));
	logger.add_datapoint("IK.fixed[l_leg_lhy]","-",&(forceMask[6]));
	logger.add_datapoint("IK.fixed[l_leg_kny]","-",&(forceMask[7]));
	logger.add_datapoint("IK.fixed[l_leg_uay]","-",&(forceMask[8]));
	logger.add_datapoint("IK.fixed[l_leg_lax]","-",&(forceMask[9]));
	logger.add_datapoint("IK.fixed[r_leg_uhz]","-",&(forceMask[10]));
	logger.add_datapoint("IK.fixed[r_leg_mhx]","-",&(forceMask[11]));
	logger.add_datapoint("IK.fixed[r_leg_lhy]","-",&(forceMask[12]));
	logger.add_datapoint("IK.fixed[r_leg_kny]","-",&(forceMask[13]));
	logger.add_datapoint("IK.fixed[r_leg_uay]","-",&(forceMask[14]));
	logger.add_datapoint("IK.fixed[r_leg_lax]","-",&(forceMask[15]));
	logger.add_datapoint("IK.fixed[l_arm_usy]","-",&(forceMask[16]));
	logger.add_datapoint("IK.fixed[l_arm_shx]","-",&(forceMask[17]));
	logger.add_datapoint("IK.fixed[l_arm_ely]","-",&(forceMask[18]));
	logger.add_datapoint("IK.fixed[l_arm_elx]","-",&(forceMask[19]));
	logger.add_datapoint("IK.fixed[l_arm_uwy]","-",&(forceMask[20]));
	logger.add_datapoint("IK.fixed[l_arm_mwx]","-",&(forceMask[21]));
	logger.add_datapoint("IK.fixed[r_arm_usy]","-",&(forceMask[22]));
	logger.add_datapoint("IK.fixed[r_arm_shx]","-",&(forceMask[23]));
	logger.add_datapoint("IK.fixed[r_arm_ely]","-",&(forceMask[24]));
	logger.add_datapoint("IK.fixed[r_arm_elx]","-",&(forceMask[25]));
	logger.add_datapoint("IK.fixed[r_arm_uwy]","-",&(forceMask[26]));
	logger.add_datapoint("IK.fixed[r_arm_mwx]","-",&(forceMask[27]));
}




bool PelvIkCon::IK(const IKcmd &cmd, double *theta_d, double *thetad_d) {
	Eigen::AngleAxisd rotVec;
	Eigen::Vector3d Rotvec;

	if (ikrs->getType() != _type)		return false;

	int rowIdx = 0;

	_A.setZero();
	_b.setZero();
	Eigen::Map<Eigen::MatrixXd> A(_A.data(),MAX_ROWS,PELV_N_SDFAST_U);
	Eigen::Map<Eigen::VectorXd> b(_b.data(),MAX_ROWS);

	//elbows *********************************************************************************
	//Y translation only
	for(int s = 0; s < 2; s++) {
		if(cmd.elbowWeight[s] > 0) {
			A.block<1,PELV_N_SDFAST_U>(rowIdx,0) = ikrs->Jelbow[s].block<PELV_N_SDFAST_U,1>(0,1).transpose() * cmd.elbowWeight[s];
			b(rowIdx) = IK_POS_RATE*(cmd.desElbow[s] - ikrs->elbow[s][YY]) * cmd.elbowWeight[s];
			rowIdx += 1;	//indexing
		}
	}
	//feet ***********************************************************************************
	//Jacobian in A
	for(int s = 0; s < 2; s++) {
		if(!cmd.skipFoot[s]) {
			A.block<6,PELV_N_SDFAST_U>(rowIdx,0) = ikrs->J[s].transpose() * IK_FOOT_WEIGHT;

			//translation
			for (int i = 0; i < 3; i++)
				b(rowIdx+i) = cmd.footd[s][i] + IK_POS_RATE*(cmd.foot[s][i] - ikrs->feet[s].w_sensor_pos[i]);

			//rotation
			Rotvec = quatMinus(ikrs->feet[s].w_q, cmd.footQ[s]);
			for (int i = 3; i < 6; i++)
				b(rowIdx+i) = cmd.footd[s][i] - IK_POS_RATE*Rotvec[i-3];

			//weight
			b.segment<6>(rowIdx) *= IK_FOOT_WEIGHT;
			rowIdx += 6;	//indexing
		}
	}


	//hands ***********************************************************************************
	for(int s = 0; s < 2; s++) {
		if(!cmd.skipHand[s]) {
			//temporary matrix makes rotation easier
			Eigen::Matrix<double, 6, PELV_N_SDFAST_U> Ahand;
			Eigen::MatrixXd bHand(6,1);

			//Jacobian in A
			Ahand = ikrs->Jhand[s].transpose();

			//translation
			for (int i = 0; i < 3; i++)
				bHand(i,0) = cmd.handd[s][i] + IK_POS_RATE*(cmd.hand[s][i] - ikrs->hand[s][i]);

			//rotation
			Rotvec = quatMinus(ikrs->handQ[s], cmd.handQ[s]);
			for (int i = 3; i < 6; i++)		bHand(i,0) = cmd.handd[s][i] - IK_POS_RATE*Rotvec[i-3];



			//translational weight
			b.segment<3>(rowIdx) = bHand.block<3, 1>(0, 0) * hand_weight_p;
			A.block<3,PELV_N_SDFAST_U>(rowIdx,0) = Ahand.block<3, PELV_N_SDFAST_U>(0,0) * hand_weight_p;


			//rotate into a basis based on the hand axis
			Eigen::Matrix3d Raxis;
			if(cmd.drillAxisMode)		Raxis = getBasis(ikrs->drillAxis[s]);
			else						Raxis = getBasis(ikrs->handAxis[s]);
			b.segment<3>(rowIdx+3) = Raxis*bHand.block<3, 1>(3, 0);
			A.block<3,PELV_N_SDFAST_U>(rowIdx+3,0) = Raxis*Ahand.block<3, PELV_N_SDFAST_U>(3,0);


			//rotational weight
			if(cmd.rotGain[s]) {
				b.segment<1>(rowIdx+3) *= cmd.rotGain[s]->x();
				b.segment<1>(rowIdx+4) *= cmd.rotGain[s]->y();
				b.segment<1>(rowIdx+5) *= cmd.rotGain[s]->z();
				A.block<1,PELV_N_SDFAST_U>(rowIdx+3,0) *= cmd.rotGain[s]->x();
				A.block<1,PELV_N_SDFAST_U>(rowIdx+4,0) *= cmd.rotGain[s]->y();
				A.block<1,PELV_N_SDFAST_U>(rowIdx+5,0) *= cmd.rotGain[s]->z();
			}
			else {
				double rollWeight = hand_weight_r_axial;
				double dRoll = std::fabs(b[rowIdx+3])/IK_POS_RATE;
				if(dRoll > hardenAxialThresh) 	rollWeight += (dRoll-hardenAxialThresh)*hardenAxialWeight;

				b.segment<1>(rowIdx+3) *= rollWeight;
				b.segment<1>(rowIdx+4) *= hand_weight_r_pitchish;
				b.segment<1>(rowIdx+5) *= hand_weight_r_yawish;
				A.block<1,PELV_N_SDFAST_U>(rowIdx+3,0) *= rollWeight;
				A.block<1,PELV_N_SDFAST_U>(rowIdx+4,0) *= hand_weight_r_pitchish;
				A.block<1,PELV_N_SDFAST_U>(rowIdx+5,0) *= hand_weight_r_yawish;
			}
			rowIdx += 6;	//indexing
		}
	}


	//CoM (translation only) *****************************************************************************
	//Jacobian in A
	double posRateC = IK_POS_RATE;
	if(posRateCOM >=0)	posRateC = posRateCOM;

	A.block<3,PELV_N_SDFAST_U>(rowIdx,0) = ikrs->Jc.transpose();
	for(int i = 0; i < 3; i++) {
		if(cmd.rootMode[i]) {	//replace with a row of the identity matrix
			A.block<1, PELV_N_SDFAST_U>(rowIdx+i,0).setZero();
			A(rowIdx+i,i) = 1.0;
		}
	}

	//translation
	for (int i = 0; i < 3; i++) {
		if(cmd.rootMode[i])		b(rowIdx+i) = cmd.rootd[i] + posRateC*(cmd.root[i] - ikrs->root[i]);
		else					b(rowIdx+i) = cmd.comd[i]  + posRateC*(cmd.com[i]  - ikrs->com[i]);
	}

	//weight
	b.segment<3>(rowIdx);
	for(int i = 0; i < 3; i++) {
		b[rowIdx+i] *= IK_COMxyz_WEIGHT[i];
		A.block<1, PELV_N_SDFAST_U>(rowIdx+i,0) *= IK_COMxyz_WEIGHT[i];
	}

	rowIdx += 3;	//indexing

	if(cmd.dropHeight) {
		b[rowIdx] = 0;
		A.block<1, PELV_N_SDFAST_U>(rowIdx,0).setZero();
		rowIdx-=1;
	}


	//Pelvis (orientation only) **************************************************************************
	//Jacobian in A (2nd half only)
	double posRateP = IK_POS_RATE;
	if(posRatePelv >= 0)	posRateP = posRatePelv;

	A.block<3,PELV_N_SDFAST_U>(rowIdx,0) = ikrs->Jpelvis.block<PELV_N_SDFAST_U,3>(0,3).transpose() * IK_PELVIS_WEIGHT;

	//rotation
	Rotvec = quatMinus(ikrs->pelvisq, cmd.pelvQ);
	for (int i = 0; i < 3; i++)
		b(rowIdx+i) = cmd.pelvW[i] - posRateP*Rotvec[i];

	//weight
	b.segment<3>(rowIdx) *= IK_PELVIS_WEIGHT;
	rowIdx += 3;	//indexing

	//Torso (orientation only) **************************************************************************
	//Jacobian in A (2nd half only)
	double posRateT = IK_POS_RATE;
	if(posRateTorso >= 0)	posRateT = posRateTorso;

	A.block<3,PELV_N_SDFAST_U>(rowIdx,0) = ikrs->Jtorso.block<PELV_N_SDFAST_U,3>(0,3).transpose() * torsoWeight;

	//rotation
	Rotvec = quatMinus(ikrs->utorsoq, cmd.torsoQ);
	for (int i = 0; i < 3; i++)
		b(rowIdx+i) = cmd.torsoW[i] - posRateT*Rotvec[i];

	//weight
	b.segment<3>(rowIdx) *= torsoWeight;

	if(hardenTorso) {
		double torsoEA[3];
		quat2EA(ikrs->utorsoq, torsoEA);
		double torsoPitch = torsoEA[1];

		double hardness = 0;
		if(torsoPitch > torsoPitchLimit[1])			hardness = torsoPitch-torsoPitchLimit[1];
		else if(torsoPitch < torsoPitchLimit[0])	hardness = torsoPitchLimit[0] - torsoPitch;

		double weightMult = (hardness*30.0+torsoWeight)/torsoWeight;
		b[rowIdx+1] *= weightMult;
		A.block<1,PELV_N_SDFAST_U>(rowIdx+1,0) *= weightMult;
	}

	rowIdx += 3;	//indexing

	//Regularization **************************************************************************
	A.block<PELV_N_SDFAST_U,PELV_N_SDFAST_U>(rowIdx,0) = Eigen::Matrix<double,PELV_N_SDFAST_U,PELV_N_SDFAST_U>::Identity() * IK_REG_WEIGHT;
	b.segment<PELV_N_SDFAST_U>(rowIdx).setZero();
	rowIdx += PELV_N_SDFAST_U;

	//Rerence pose ****************************************************************************
	A.block<PELV_N_JOINTS,PELV_N_JOINTS>(rowIdx, 6) = Eigen::Matrix<double,PELV_N_JOINTS,PELV_N_JOINTS>::Identity() * IK_REF_WEIGHT;
	for(int i = 0; i < PELV_N_JOINTS; i++)	b[rowIdx+i] = IK_REF_GAIN*(refJoints[i]-ikrs->joints[i]);
	rowIdx += PELV_N_JOINTS;

	if(ikrs->joints[A_L_ARM_ELX] < elbowThresh && elbowWeight > 0.0) {
		double weight = elbowWeight*(elbowThresh-ikrs->joints[A_L_ARM_ELX]);
		A(rowIdx, 6+A_L_ARM_ELX) = weight;
		b[rowIdx] = 0.5*weight;
		rowIdx++;
		Logger::setTmpOut(0, weight);
	}
	else Logger::setTmpOut(0,0.0);

	if(ikrs->joints[A_R_ARM_ELX] > -elbowThresh && elbowWeight > 0.0) {
		double weight = elbowWeight*(ikrs->joints[A_R_ARM_ELX]+elbowThresh);
		A(rowIdx, 6+A_R_ARM_ELX) = weight;
		b[rowIdx] = -0.5*weight;
		rowIdx++;
		Logger::setTmpOut(1, weight);
	}
	else Logger::setTmpOut(1,0.0);

	/////////////////////////////////////////////////////////
	const int N_VARS = PELV_N_SDFAST_U;
	const int N_EQ_CON = PELV_N_JOINTS+3;
	const int N_INEQ_CON = PELV_N_JOINTS*2 + 2 + 2 + 4;

	Eigen::Matrix<double,N_VARS,1> x;

	Eigen::Matrix<double,N_VARS,N_VARS> G =	A.topRows(rowIdx).transpose()*A.topRows(rowIdx);
	Eigen::Matrix<double,N_VARS,1> g0 =	-A.topRows(rowIdx).transpose()*b.topRows(rowIdx);

	Eigen::Matrix<double,N_EQ_CON, N_VARS> CE;
	Eigen::Matrix<double,N_EQ_CON,1> ce0;
	CE.setZero();
	ce0.setZero();

	Eigen::Matrix<double,N_INEQ_CON,N_VARS> CI;
	Eigen::Matrix<double,N_INEQ_CON,1> ci0;
	CI.setZero();
	ci0.setZero();

	//Begin Equality Constraints
	/******************************************************************************/

	int ceInd = 0;
	//manually force individual joints to specific values
	for(int i = 0; i < PELV_N_JOINTS; i++) {
		if(forceMask[i]) {
			CE(ceInd, i+6) = ikrs->timeStep;
			ce0(ceInd) = ikrs->joints[i] - RobotState::putInBounds(forceQ[i], i);
			ceInd++;
		}
	}

	//require that the hands maintain perfect relative position
	if(cmd.relHands) {
		if(cmd.skipHand[LEFT] || cmd.skipHand[RIGHT]) {
			fprintf(stderr, "Can't do relative hands if skipping one hand\n");
		}
		else {
			//coefficient sets the time constant
			CE.block<3,PELV_N_SDFAST_U>(ceInd,0) = 1.0*(ikrs->Jhand[LEFT].block<PELV_N_SDFAST_U,3>(0,0).transpose() - ikrs->Jhand[RIGHT].block<PELV_N_SDFAST_U,3>(0,0).transpose());
			for(int i = 0; i < 3; i++)	ce0(ceInd+i) = (ikrs->hand[LEFT][i]-ikrs->hand[RIGHT][i]) - (cmd.hand[LEFT][i]-cmd.hand[RIGHT][i]);
			ceInd += 3;
		}
	}




	//Begin Inequality Constraints
	/*****************************************************************/


	int IEind = 0;
	// joint limit:
	for (int i = 0; i < PELV_N_JOINTS; i++) {
		//  qd*dt >= low - q
		CI(2*i, 6+i) = ikrs->timeStep;
		if(i == 4)		ci0(2*i) = -RobotState::hipZlim(ikrs->joints[6]) + ikrs->joints[i];
		else if(i == 7 && cmd.noLockKnees)		ci0(2*i) = -0.25 + ikrs->joints[i];
		else if(i == 13 && cmd.noLockKnees)		ci0(2*i) = -0.25 + ikrs->joints[i];
		else			ci0(2*i) = -RobotState::jointsLimit[0][i] + ikrs->joints[i];

		// -qd*dt >= q - up
		CI(2*i+1, 6+i) = -ikrs->timeStep;
		if(i == 10)		ci0(2*i+1) = -RobotState::hipZlim(ikrs->joints[12]) - ikrs->joints[i];
		else			ci0(2*i+1) = RobotState::jointsLimit[1][i] - ikrs->joints[i];
	}
	IEind += PELV_N_JOINTS*2;

	//elbow constraints (constraing elbows in the Y direction)
	if(cmd.elbowLims[LEFT][2]!=0.0) {
		CI.block<1,PELV_N_SDFAST_U>(IEind,0) = -(ikrs->Jelbow[LEFT].block<PELV_N_SDFAST_U,1>(0,0).transpose()*cmd.elbowLims[LEFT][0] + ikrs->Jelbow[LEFT].block<PELV_N_SDFAST_U,1>(0,1).transpose()*cmd.elbowLims[LEFT][1])*0.5;
		ci0(IEind) = -ikrs->elbow[LEFT][XX]*cmd.elbowLims[LEFT][0] - ikrs->elbow[LEFT][YY]*cmd.elbowLims[LEFT][1] - cmd.elbowLims[LEFT][2];
		IEind++;
	}

	if(cmd.elbowLims[RIGHT][2]!=0.0) {
		CI.block<1,PELV_N_SDFAST_U>(IEind,0) = (ikrs->Jelbow[RIGHT].block<PELV_N_SDFAST_U,1>(0,0).transpose()*cmd.elbowLims[RIGHT][0] + ikrs->Jelbow[RIGHT].block<PELV_N_SDFAST_U,1>(0,1).transpose()*cmd.elbowLims[RIGHT][1])*0.5;
		ci0(IEind) = ikrs->elbow[RIGHT][XX]*cmd.elbowLims[RIGHT][0] + ikrs->elbow[RIGHT][YY]*cmd.elbowLims[RIGHT][1] + cmd.elbowLims[RIGHT][2];
		IEind++;
	}

	//might need to handle knee specially if either foot is going up or root is going down
	//check if root is going down
	bool dropRoot= false;
	if(cmd.rootMode[ZZ]) {
		if(cmd.rootd[ZZ] < 0)						dropRoot = true;
		if(cmd.root[ZZ] < ikrs->root[ZZ]+.001)		dropRoot = true;
	}
	else {
		if(cmd.comd[ZZ] < 0)						dropRoot = true;
		if(cmd.com[ZZ] < ikrs->com[ZZ]+.001)		dropRoot = true;
	}

	if(cmd.dropHeight)		dropRoot = false;

	//unlock the knee
	for(int s = 0; s < 2; s++) {
		if((dropRoot || (cmd.footd[s][ZZ] > 0)  ||  (cmd.foot[s][ZZ] > ikrs->feet[s].w_sensor_pos[ZZ]+.005)) && (ikrs->joints[7+6*s]< 0.25) && !forceMask[7+6*s]) {	//unlock the knee
			CI(IEind, 6+7+6*s) = 1.0;
			ci0(IEind) = -1.0;
			IEind++;
		}
	}

	//don't let the CoM leave the support box
	if(cmd.constrainCoM) {
		Eigen::Vector3d fc = ikrs->getFootCenter();
		double boundary[4];
		boundary[0] = fc[XX] - 0.0;
		boundary[1] = fc[XX] + 0.09;
		boundary[2] = ikrs->feet[RIGHT].w_sensor_pos[YY]+0.04;
		boundary[3] = ikrs->feet[LEFT].w_sensor_pos[YY]-0.04;

		CI.block<1,PELV_N_SDFAST_U>(IEind+0,0) =  ikrs->Jc.block<PELV_N_SDFAST_U,1>(0,0).transpose()*ikrs->timeStep;
		CI.block<1,PELV_N_SDFAST_U>(IEind+1,0) = -ikrs->Jc.block<PELV_N_SDFAST_U,1>(0,0).transpose()*ikrs->timeStep;
		CI.block<1,PELV_N_SDFAST_U>(IEind+2,0) =  ikrs->Jc.block<PELV_N_SDFAST_U,1>(0,1).transpose()*ikrs->timeStep;
		CI.block<1,PELV_N_SDFAST_U>(IEind+3,0) = -ikrs->Jc.block<PELV_N_SDFAST_U,1>(0,1).transpose()*ikrs->timeStep;

		ci0(IEind+0) =  ikrs->com[XX] - boundary[0];
		ci0(IEind+1) = -ikrs->com[XX] + boundary[1];
		ci0(IEind+2) =  ikrs->com[YY] - boundary[2];
		ci0(IEind+3) = -ikrs->com[YY] + boundary[3];
		IEind += 4;
	}


	/*
  min 0.5 * x G x + g0 x
  s.t.
    CE^T x + ce0 = 0
    CI^T x + ci0 >= 0 
	 */

	bool recompute = true;
	computeCount = 0;



	QPval = Eigen::solve_quadprog<N_VARS,N_EQ_CON,N_INEQ_CON>(G, g0, CE.transpose(), ce0, CI.transpose(), ci0, x);
	if(QPval == std::numeric_limits<double>::infinity())	Logger::setErrEW(1);


	// integrate one step
	double qdot[PELV_N_SDFAST_Q] = {0};
	double q[PELV_N_SDFAST_Q] = {0};
	dvec_copy(q, ikrs->getSDFState(), PELV_N_SDFAST_Q);
	ikrs->model->sdu2qdot(x.data(), qdot);

	for (int i = 0; i < PELV_N_SDFAST_Q; i++)
		q[i] += ikrs->timeStep * qdot[i];

	// update Robot State
	double filtA = 0.05;
	ikrs->time += ikrs->timeStep;
	dvec_copy(ikrs->root, q, 3);
	for(int i = 0; i < 3; i++)	ikrs->rootd[i] = (1-filtA)*ikrs->rootd[i] + filtA*qdot[i];
	ikrs->rootq = Eigen::Quaterniond(q[PELV_S_Q3], q[PELV_S_Q0], q[PELV_S_Q1], q[PELV_S_Q2]);
	ikrs->rootq.normalize();
	for(int i = 0; i < 3; i++)	ikrs->root_b_w[i] = (1-filtA)*ikrs->root_b_w[i] + filtA*qdot[i+3];
	dvec_copy(ikrs->joints, q+6, PELV_N_JOINTS);

	for(int i = 0; i < PELV_N_JOINTS; i++)	ikrs->jointsd[i] = (1-filtA)*ikrs->jointsd[i] + filtA*qdot[i+6];
	ikrs->computeSDFvars();	//forward kinematics, Jacobians, etc.

	//output
	if(theta_d)		dvec_copy(theta_d, q+6, PELV_N_JOINTS);
	if(thetad_d)	dvec_copy(thetad_d, qdot+6, PELV_N_JOINTS);

	//error checking
	outOfLimits = -1;
	for(int i = 0; i < N_JOINTS; i++) {
		if(!ikrs->inBounds(i))	 {
			outOfLimits = i;
			break;	//get first one
		}
	}

	return true;
}



void PelvIkCon::getCommand(double *theta_d, double *thetad_d) {
	if(theta_d)		dvec_copy(theta_d, ikrs->joints, PELV_N_JOINTS);
	if(thetad_d)	dvec_copy(thetad_d, ikrs->jointsd, PELV_N_JOINTS);
}

