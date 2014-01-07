/*
 *  Copyright 2013 Sasanka Nagavalli
 *  Based on code by Siyuan Feng
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include <cmath>
#include <cstdlib>

#include <fstream>
#include <iostream>

#include "drc_pelvis_defines.h"
#include "id_controller.hpp"
#include "eiquadprog.hpp"
#include "Utils.hpp"
#include "Eigen_utils.hpp"
#include "Transmission.h"

//in case we need a link to make it dump later
static BatchLogger *logPoint;

const int N_Q = 35;
const int N_U = 34;

bool isDown(int side, int contactState) {
	if(side == LEFT) 	return (contactState==DSc) || (contactState==SSL);
	else				return (contactState==DSc) || (contactState==SSR);
}

bool isHandContact(int side, int hc) {
	if(side == LEFT)	return (hc == BOTH_HANDS) || (hc == LEFT_HAND);
	else				return (hc == BOTH_HANDS) || (hc == RIGHT_HAND);
}


double PelvIdCon::getFz(int side) {
	return _forceOld[ZZ+6*side];
}

double PelvIdCon::getCoPx(int side) {
	if(_forceOld[ZZ+6*side] == 0)	return 0;
	return -_forceOld[YY+3+6*side]/_forceOld[ZZ+6*side];
}

std::ofstream outputFile;

bool PelvIdCon::ID(const RobotState &rs, const IDcmd &cmd, double *torque) {

	// different model
	if (rs.getType() != _type)		return false;

	double sup_box[2][4];
	rs.feet[LEFT].getBoundingBox(Foot::RefSensor, &(sup_box[LEFT][0]));
	rs.feet[RIGHT].getBoundingBox(Foot::RefSensor, &(sup_box[RIGHT][0]));

	for(int i = 0; i < 2; i++) {
		if(cmd.heelSupport[i] > 0.0) {
			sup_box[i][1] = sup_box[i][0]-cmd.heelSupport[i];
			sup_box[i][0] +=  0.02;
		}
	}

	static Eigen::Quaterniond footq_h[LR] = {Eigen::Quaterniond::Identity(), Eigen::Quaterniond::Identity()};
	const double e = 0.9;

	int nForces = 0;

	// 2 * [Fxyz,Txyz]
	switch (cmd.contactState) {
	case DSl:
	case DSr:
	case DSc: nForces = 12; break;
	case SSL:
	case SSR: nForces = 6;  break;
	default:
		assert(0);
	}

	// nVars is [u_dot, tau, 2*contact_f]'
	const int nVars = PELV_N_SDFAST_U + PELV_N_JOINTS + nForces + 6;
	const int nEqCon = PELV_N_SDFAST_U;
	const int nIneqCon = 9*nForces/6;
	const int forceStart = PELV_N_SDFAST_U + PELV_N_JOINTS;
	const int hfStart = forceStart + nForces;
	const int nhForces = 6;
	const int torqueStart = PELV_N_SDFAST_U;

	_A.setZero();
	_b.setZero();
	_X.setZero();
	_AD.setZero();
	_Rot.setZero();
	_AF.setZero();
	Eigen::Map<Eigen::MatrixXd> A(_A.data(),MAX_ROWS,nVars);
	Eigen::Map<Eigen::VectorXd> b(_b.data(),MAX_ROWS);

	// rotate world force into local frame
	Eigen::Map<Eigen::MatrixXd> Rot(_Rot.data(),nForces,nForces);

	Eigen::Matrix<double,6,6> Rot_left;
	Eigen::Matrix<double,6,6> Rot_right;

	//Eigen::AngleAxisd tmpaa(rs.feet[LEFT].w_q);
	//Eigen::Quaterniond q(Eigen::AngleAxisd(tmpaa.angle(), Eigen::Vector3d::UnitZ()));
	//Eigen::Quaterniond q = cmd.footq_d[LEFT];
	Eigen::Quaterniond q(Eigen::AngleAxisd(rs.feet[LEFT].w_sensor_pos[5], Eigen::Vector3d::UnitZ()));
	Eigen::Matrix3d footRot = q.inverse().normalized().toRotationMatrix();

	Rot_left.setZero();
	Rot_left.block<3,3>(0,0) = footRot;
	Rot_left.block<3,3>(3,3) = footRot;

	//tmpaa = Eigen::AngleAxisd(rs.feet[RIGHT].w_q);
	//q = Eigen::Quaterniond(Eigen::AngleAxisd(tmpaa.angle(), Eigen::Vector3d::UnitZ()));
	//q = cmd.footq_d[RIGHT];
	q = Eigen::Quaterniond(Eigen::AngleAxisd(rs.feet[RIGHT].w_sensor_pos[5], Eigen::Vector3d::UnitZ()));
	footRot = q.inverse().normalized().toRotationMatrix();
	Rot_right.setZero();
	Rot_right.block<3,3>(0,0) = footRot;
	Rot_right.block<3,3>(3,3) = footRot;

	switch (cmd.contactState) {
	case DSl:
	case DSr:
	case DSc:
		Rot.block<6,6>(0,0) = Rot_left;
		Rot.block<6,6>(6,6) = Rot_right;
		break;
	case SSL:
		Rot.block<6,6>(0,0) = Rot_left;
		break;
	case SSR:
		Rot.block<6,6>(0,0) = Rot_right;
		break;
	default:
		assert(0);
	}

	//rs does not store a continuous sdfast state (silly Siyuan?)
	Eigen::Matrix<double,PELV_N_SDFAST_U,1> qdot;
	qdot << Eigen::Map<const Eigen::Matrix<double,6,1> >(rs.rootd),Eigen::Map<const Eigen::Matrix<double,PELV_N_JOINTS,1> >(rs.jointsd);

	int rowIdx = 0;


	// Qdd Regularization
	/***************************
	 * AD = [I, 0, 0, 0];
	 * bD = 0;
	 ****************************/
	if(QDD_REG_WEIGHT != 0 || FIX_QDD_WEIGHT != 0) {
		b.segment<N_U>(rowIdx).setZero();	//could put desired accelerations here
		for (int i = 0; i < N_U; i++) {
			A(rowIdx+i,i) = 1;
		}

		A.block(rowIdx,0,N_U,N_U) *= QDD_REG_WEIGHT;
		b.segment<N_U>(rowIdx) *= QDD_REG_WEIGHT;		//not necessary

		//overwrite with fixed DoFs
		for(int i = 0; i < N_U; i++) {
			if(fixQDDmask[i]) {
				A(rowIdx+i, i) = 		1.0 * 		FIX_QDD_WEIGHT;
				b(rowIdx+i) = 			fixQdd[i] *	FIX_QDD_WEIGHT;
			}
		}
		rowIdx += N_U;
	}

	if((COMDD_XY_WEIGHT != 0) || (COMDD_Z_WEIGHT != 0)) {
		if(COM_BY_ACCEL) {
			//=== Desired COM qdd
			/***************************
			 * AcomQdd = [Jcom, 0, 0, 0]
			 * bcomQdd = [comdd_d + -Jd * qd];
			 ****************************/

			double z = rs.com[ZZ] - (rs.feet[LEFT].w_sensor_pos[ZZ] + rs.feet[RIGHT].w_sensor_pos[ZZ]) / 2.;
			A.block<3,PELV_N_SDFAST_U>(rowIdx,0) = rs.Jc.transpose();

			b.segment<3>(rowIdx) = Eigen::Map<const Eigen::Matrix<double,3,1> >(cmd.comdd);
			b.segment<3>(rowIdx) += -rs.Jcd.transpose()*qdot*QD_JD_USE_FRAC;
			b(rowIdx+XX) -= _magicCop[XX]*GRAVITY / z;
			b(rowIdx+YY) -= _magicCop[YY]*GRAVITY / z;

			//split weight into XY and Z
			A.block<2,PELV_N_SDFAST_U>(rowIdx,0) *= COMDD_XY_WEIGHT*rs.m*GRAVITY;
			b.segment<2>(rowIdx) *= COMDD_XY_WEIGHT*rs.m*GRAVITY;
			A.block<1,PELV_N_SDFAST_U>(rowIdx+2,0) *= COMDD_Z_WEIGHT*rs.m*GRAVITY;
			b.segment<1>(rowIdx+2) *= COMDD_Z_WEIGHT*rs.m*GRAVITY;

			rowIdx += 3;
		}
		else {	//COM_BY_FORCE
			//=== Sum of forces on COM
			/***************************
			 * AcomF = [0, 0, [I 0], [I 0]]
			 * bcomF = [comdd_d + mg + task_force];
			 ****************************/

			double z = rs.com[ZZ] - (rs.feet[LEFT].w_sensor_pos[ZZ] + rs.feet[RIGHT].w_sensor_pos[ZZ]) / 2.;
			b.segment<3>(rowIdx) = Eigen::Map<const Eigen::Matrix<double,3,1> >(cmd.comdd);

			switch (cmd.contactState) {
			case DSl:
			case DSr:
			case DSc:
				A.block<3,3>(rowIdx,forceStart).setIdentity();
				A.block<3,3>(rowIdx,forceStart+6).setIdentity();
				break;
			case SSL:
			case SSR:
				A.block<3,3>(rowIdx,forceStart).setIdentity();
				break;
			default: assert(0); // BUG!
			}
			//account for hand forces
			for(int s = 0; s < 2; s++) 		if(isHandContact(s, cmd.handContact))	A.block<3,3>(rowIdx, hfStart+3*s).setIdentity();

			b.segment<3>(rowIdx) = Eigen::Map<const Eigen::Vector3d>(cmd.comdd);
			b(rowIdx+XX) -= _magicCop[XX]*GRAVITY / z;
			b(rowIdx+YY) -= _magicCop[YY]*GRAVITY / z;
			b(rowIdx+ZZ) += GRAVITY;
			b.segment<3>(rowIdx) *= rs.m;

			//split weight into XY and Z
			A.block(rowIdx,forceStart, 2, nForces) *= COMDD_XY_WEIGHT;
			b.segment<2>(rowIdx) *= COMDD_XY_WEIGHT;
			A.block(rowIdx+2,forceStart, 1, nForces) *= COMDD_Z_WEIGHT;
			b.segment<1>(rowIdx+2) *= COMDD_Z_WEIGHT;

			rowIdx += 3;

		}
	}

	//=== Desired utorso orientation
	/***************************
	 * AcomQdd = [Jtorso, 0, 0, 0]
	 * bcomQdd = [torsodd_d + -Jd * qd];
	 ****************************/
	if(ANGDD_T_WEIGHT != 0) {
		A.block<3,PELV_N_SDFAST_U>(rowIdx,0) = rs.Jtorso.block<PELV_N_SDFAST_U,3>(0,3).transpose();

		b.segment<3>(rowIdx) = Eigen::Map<const Eigen::Matrix<double,3,1> >(cmd.torsoWd);
		b.segment<3>(rowIdx) += -rs.Jtorsod.block<PELV_N_SDFAST_U,3>(0,3).transpose()*qdot * QD_JD_USE_FRAC;

		b.segment<3>(rowIdx) *= ANGDD_T_WEIGHT;
		A.block<3,PELV_N_SDFAST_U>(rowIdx,0) *= ANGDD_T_WEIGHT;

		rowIdx += 3;
	}

	//=== Desired pelvis orientation
	/***************************
	 * AcomQdd = [Jpelvis, 0, 0, 0]
	 * bcomQdd = [pelvisdd_d + -Jd * qd];
	 ****************************/
	if(ANGDD_P_WEIGHT != 0) {
		A.block<3,PELV_N_SDFAST_U>(rowIdx,0) = rs.Jpelvis.block<PELV_N_SDFAST_U,3>(0,3).transpose();

		b.segment<3>(rowIdx) = Eigen::Map<const Eigen::Matrix<double,3,1> >(cmd.pelvWd);
		b.segment<3>(rowIdx) += -rs.Jpelvisd.block<PELV_N_SDFAST_U,3>(0,3).transpose()*qdot * QD_JD_USE_FRAC;

		b.segment<3>(rowIdx) *= ANGDD_P_WEIGHT;
		A.block<3,PELV_N_SDFAST_U>(rowIdx,0) *= ANGDD_P_WEIGHT;

		rowIdx += 3;
	}

	//=== feet accelerations
	/***************************
	 * AC = [JL,     0,     0;
	 *       JR,     0,     0]
	 * bC = [footdd_d[LEFT] - Jd*qd;
	 *       footdd_d[RIGHT] - Jd*qd]
	 ****************************/
	if(FOOTDD_WEIGHT != 0) {
		for(int side = 0; side < 2; side++) {
			A.block<6,PELV_N_SDFAST_U>(rowIdx,0) = rs.J[side].transpose();

			b.segment<6>(rowIdx) = Eigen::Map<const Eigen::Matrix<double,6,1> >(&(cmd.footdd[side][0]));
			b.segment<6>(rowIdx) += -rs.Jd[side].transpose()*qdot * QD_JD_USE_FRAC;

			A.block<6,PELV_N_SDFAST_U>(rowIdx,0) *= FOOTDD_WEIGHT;
			b.segment<6>(rowIdx) *= FOOTDD_WEIGHT;
			if(!isDown(side, cmd.contactState)) {
				A.block<6,PELV_N_SDFAST_U>(rowIdx,0) *= SWING_MULT_WEIGHT;
				b.segment<6>(rowIdx) *= SWING_MULT_WEIGHT;
			}

			rowIdx += 6;
		}
	}

	//=== hand accelerations
	/***************************
	 * AC = [JL,     0,     0;
	 *       JR,     0,     0]
	 * bC = [handdd_d[LEFT] - Jd*qd;
	 *       handdd_d[RIGHT] - Jd*qd]
	 ****************************/
	if(HANDDD_WEIGHT != 0) {
		for(int side = 0; side < 2; side++) {
			if(!cmd.skipHand[side]) {
				A.block<6,PELV_N_SDFAST_U>(rowIdx,0) = rs.Jhand[side].transpose();

				b.segment<6>(rowIdx) = Eigen::Map<const Eigen::Matrix<double,6,1> >(&(cmd.handdd[side][0]));
				b.segment<6>(rowIdx) += -rs.Jhandd[side].transpose()*qdot * QD_JD_USE_FRAC;

				A.block<2,PELV_N_SDFAST_U>(rowIdx,0) *= handWeightP;
				b.segment<3>(rowIdx) *= handWeightP;
				A.block<3,PELV_N_SDFAST_U>(rowIdx+3,0) *= handWeightR;
				b.segment<3>(rowIdx+3) *= handWeightR;

				rowIdx += 6;
			}
		}
	}


	// ======= minimize torques
	/***************************
	 * AT = [0, I, 0, 0]
	 * bT = 0;
	 ****************************/
	if(TAU_REG_WEIGHT != 0) {
		A.block<PELV_N_JOINTS,PELV_N_JOINTS>(rowIdx,torqueStart).setIdentity();
		b.segment<PELV_N_JOINTS>(rowIdx).setZero();

		A.block<PELV_N_JOINTS,PELV_N_JOINTS>(rowIdx,torqueStart) *= TAU_REG_WEIGHT;

		rowIdx += PELV_N_JOINTS;
	}

	//======= minimize change in torques
	/***************************
	 * AprevTrq = [0, I, 0, 0, 0]
	 * bprevTrq = [_torqueOld];
	 ****************************/
	if (DTAU_REG_WEIGHT != 0) {
		//if (DTauWeight != 0 && !_torqueOld.isZero()) {
		A.block<PELV_N_JOINTS,PELV_N_JOINTS>(rowIdx,torqueStart).setIdentity();
		b.segment<PELV_N_JOINTS>(rowIdx) = _torqueOld;

		A.block<PELV_N_JOINTS,PELV_N_JOINTS>(rowIdx,torqueStart) *= DTAU_REG_WEIGHT;
		b.segment<PELV_N_JOINTS>(rowIdx) *= DTAU_REG_WEIGHT;

		rowIdx += PELV_N_JOINTS;
	}


	//======= minimize contact forces
	/***************************
	 * AcontactF = [0, 0, I, 0;
	 *              0, 0, 0, I]
	 * bcontactF = [0;0];
	 ****************************/
	if (F_REG_WEIGHT != 0) {
		A.block(rowIdx,forceStart,nForces,nForces).setIdentity();
		b.segment(rowIdx,nForces).setZero();
		switch (cmd.contactState) {
		case DSl:
		case DSr:
		case DSc:
			if(cmd.wl == -1) {
				b(rowIdx+ZZ) = 0;
				b(rowIdx+ZZ+6) = 0;
			}
			else {
				b(rowIdx+ZZ) = cmd.wl*rs.m*GRAVITY;
				b(rowIdx+ZZ+6) = (1-cmd.wl)*rs.m*GRAVITY;
			}
			A(rowIdx+5, forceStart+5) *= 100;
			A(rowIdx+11, forceStart+11) *= 100;
			break;
		case SSL:
		case SSR:
			b(rowIdx+ZZ) = rs.m*GRAVITY;
			A(rowIdx+5, forceStart+5) *= 100;
			break;
		default: assert(0); // BUG!
		}

		A.block(rowIdx,forceStart,nForces,nForces) *= F_REG_WEIGHT;
		b.segment(rowIdx,nForces) *= F_REG_WEIGHT;

		rowIdx += nForces;
	}

	//======= minimize hand contact forces
	/***************************
	 * AcontactF = [0, 0, I, 0;
	 *              0, 0, 0, I]
	 * bcontactF = [0;0];
	 ****************************/
	if (F_REG_WEIGHT != 0) {
		A.block<nhForces,nhForces>(rowIdx,hfStart).setIdentity();
		A.block<nhForces,nhForces>(rowIdx,hfStart) *= FH_REG_WEIGHT;

		rowIdx += nForces;
	}

	//======= minimize contact forces changes
	/***************************
	 * AcontactF = [0, 0, I, 0;
	 *              0, 0, 0, I]
	 * bcontactF = [_oldForce];
	 ****************************/
	if (DF_REG_WEIGHT != 0) {
		A.block(rowIdx,forceStart,nForces,nForces).setIdentity();
		switch (cmd.contactState) {
		case DSl:
		case DSr:
		case DSc:
			b.segment<12>(rowIdx) = _forceOld;
			break;
		case SSL:
			b.segment<6>(rowIdx) = _forceOld.segment<6>(0);
			break;
		case SSR:
			b.segment<6>(rowIdx) = _forceOld.segment<6>(6);
			break;
		default: assert(0); // BUG!
		}

		A.block(rowIdx,forceStart,nForces,nForces) *= DF_REG_WEIGHT;
		b.segment(rowIdx,nForces) *= DF_REG_WEIGHT;

		rowIdx += nForces;
	}


	//=== weight distribution
	/***************************
	 * Fzl/(Fzl+Fzr) = w
	 ****************************/
	if (isDS(cmd.contactState) && (WL_WEIGHT != 0) && (cmd.wl != -1)) {
		A(rowIdx,forceStart+2) = 1.0 - cmd.wl;
		A(rowIdx,forceStart+6+2) = -cmd.wl;

		b.segment<1>(rowIdx).setZero();

		A.block(rowIdx,forceStart,1,nForces) *= WL_WEIGHT;

		rowIdx += 1;
	}


	//======= desired center of pressure
	/***************************
	 * (FLZ+FRZ)*cop_d = FLZ*(PL + COPL) + FRZ*(PR + COPR)
	 * FLZ*(PL + COPL - cop_d) + FRZ*(PR + COPR - cop_d) = 0
	 ****************************/
	if(copWeight != 0) {
		double llCoP[2] = {cmd.cop[XX] + _magicCop[XX], cmd.cop[YY] + _magicCop[YY]};

		switch (cmd.contactState) {
		case DSl:
		case DSr:
		case DSc:
			A(rowIdx,forceStart+2) = rs.feet[LEFT].w_sensor_pos[XX] - llCoP[XX];
			A(rowIdx,forceStart+4) = -1;
			A(rowIdx,forceStart+8) = rs.feet[RIGHT].w_sensor_pos[XX] - llCoP[XX];
			A(rowIdx,forceStart+10) = -1;

			A(rowIdx+1,forceStart+2) = rs.feet[LEFT].w_sensor_pos[YY] - llCoP[YY];
			A(rowIdx+1,forceStart+3) = 1;
			A(rowIdx+1,forceStart+8) = rs.feet[RIGHT].w_sensor_pos[YY] - llCoP[YY];
			A(rowIdx+1,forceStart+9) = 1;

			b(rowIdx) = 0;
			b(rowIdx+1) = 0;

			A.block(rowIdx,forceStart,2,nForces) *= COP_WEIGHT;
			b.segment<2>(rowIdx) *= COP_WEIGHT;

			rowIdx += 2;

			break;
		case SSL:
			A(rowIdx,forceStart+2) = rs.feet[LEFT].w_sensor_pos[XX] - llCoP[XX];
			A(rowIdx,forceStart+4) = -1;
			A(rowIdx+1,forceStart+2) = rs.feet[LEFT].w_sensor_pos[YY] - llCoP[YY];
			A(rowIdx+1,forceStart+3) = 1;

			b(rowIdx) = 0;
			b(rowIdx+1) = 0;

			A.block(rowIdx,forceStart,2,nForces) *= COP_WEIGHT;
			b.segment<2>(rowIdx) *= COP_WEIGHT;

			rowIdx += 2;
			break;

		case SSR:
			A(rowIdx,forceStart+2) = rs.feet[RIGHT].w_sensor_pos[XX] - llCoP[XX];
			A(rowIdx,forceStart+4) = -1;
			A(rowIdx+1,forceStart+2) = rs.feet[RIGHT].w_sensor_pos[YY] - llCoP[YY];
			A(rowIdx+1,forceStart+3) = 1;

			b(rowIdx) = 0;
			b(rowIdx+1) = 0;

			A.block(rowIdx,forceStart,2,nForces) *= COP_WEIGHT;
			b.segment<2>(rowIdx) *= COP_WEIGHT;

			rowIdx += 2;
			break;
		default: assert(0); // BUG!
		}
	}

	//=======
	// dynamics equations
	/***************************
	 * AD = [M, -I, -JL^T, -JR^T];
	 * bD = tau
	 ****************************/
	Eigen::Map<Eigen::MatrixXd> AD(_AD.data(),PELV_N_SDFAST_U,nVars);
	AD.topLeftCorner<PELV_N_SDFAST_U,PELV_N_SDFAST_U>() = rs.M;
	AD.block<PELV_N_SDFAST_U,PELV_N_JOINTS>(0,PELV_N_SDFAST_U) <<
			Eigen::Matrix<double,6,PELV_N_JOINTS>::Zero(),
			-Eigen::Matrix<double,PELV_N_JOINTS,PELV_N_JOINTS>::Identity();
	switch (cmd.contactState) {
	case DSl:
	case DSr:
	case DSc: AD.block<PELV_N_SDFAST_U, 12>(0, forceStart) << -rs.J[LEFT], -rs.J[RIGHT]; break;
	case SSL: AD.block<PELV_N_SDFAST_U, 6>(0, forceStart) = -rs.J[LEFT];  break;
	case SSR: AD.block<PELV_N_SDFAST_U, 6>(0, forceStart)= -rs.J[RIGHT]; break;
	default: assert(0); // BUG!
	}
	for(int s = 0; s < 2; s++) {
		if(isHandContact(s, cmd.handContact)) {
			AD.block<PELV_N_SDFAST_U, 3>(0, hfStart+3*s) = -rs.Jhand[s].leftCols<3>();
		}
	}

	//Eigen::Map<const Eigen::Matrix<double,PELV_N_SDFAST_U,1> > bD(rs.nonLin);
	Eigen::Matrix<double,PELV_N_SDFAST_U,1> bD = rs.nonLin;
	bD.segment<6>(0) += _magicFT;

	//=======
	// cop and friction constraints
	double MU = FRICTION_COEF;
	double DX_toe, DX_heel, DY_left, DY_right;

	Eigen::Map<Eigen::MatrixXd> AF(_AF.data(),nIneqCon,nForces);
	AF.setZero();

	switch (cmd.contactState) {
	case DSl:
	case DSr:
	case DSc:
		DX_toe = sup_box[LEFT][0];
		DX_heel = sup_box[LEFT][1];
		DY_left = sup_box[LEFT][2];
		DY_right = sup_box[LEFT][3];
		// COP constraints
		AF.block<4,6>(0,0) <<
				0, 0, DX_toe, 0, 1, 0,     // DX_t*FLZ + MLY >= 0
				0, 0, -DX_heel, -0, -1, 0, // -DX_h*FLZ - MLY >= 0
				0, 0, DY_left, -1, 0, 0,   // DY_l*FLZ - MLX >= 0
				0, 0, -DY_right, 1, -0, 0; // -DY_r*FLZ + MLX >= 0
		DX_toe = sup_box[RIGHT][0];
		DX_heel = sup_box[RIGHT][1];
		DY_left = sup_box[RIGHT][2];
		DY_right = sup_box[RIGHT][3];
		AF.block<4,6>(4,6) <<
				0, 0, DX_toe, 0, 1, 0,     // DX_t*FRZ + MRY >= 0
				0, 0, -DX_heel, -0, -1, 0, // -DX_h*FRZ - MRY >= 0
				0, 0, DY_left, -1, 0, 0,   // DY_l*FRZ - MRX >= 0
				0, 0, -DY_right, 1, -0, 0; // -DY_r*FRZ + MRX >= 0
		AF(8,2)=1;                     // FLZ >= 0
		AF(9,8)=1;                     // FRZ >= 0

		// Friction constraints
		AF.block<4,3>(10,0) <<
				1, 0, MU,                  // MU*FLZ + FLX >= 0
				-1, 0, MU,                 // MU*FLZ - FLX >= 0
				0, 1, MU,                  // MU*FLZ + FLY >= 0
				0, -1, MU;                 // MU*FLZ - FLY >= 0
		AF.block<4,3>(14,6) <<
				1, 0, MU,                  // MU*FRZ + FRX >= 0
				-1, 0, MU,                 // MU*FRZ - FRX >= 0
				0, 1, MU,                  // MU*FRZ + FRY >= 0
				0, -1, MU;                 // MU*FRZ - FRY >= 0

		AF = AF * Rot;
		break;
	case SSL:
		DX_toe = sup_box[LEFT][0];
		DX_heel = sup_box[LEFT][1];
		DY_left = sup_box[LEFT][2];
		DY_right = sup_box[LEFT][3];
		// COP constraints
		AF.block<4,6>(0,0) <<
				0, 0, DX_toe, 0, 1, 0,     // DX_t*FZ + MY >= 0
				0, 0, -DX_heel, -0, -1, 0,  // -DX_h*FZ - MY >= 0
				0, 0, DY_left, -1, 0, 0,   // DY_l*FZ - MX >= 0
				0, 0, -DY_right, 1, -0, 0;  // -DY_r*FZ + MX >= 0
		AF(4,2)=1;                     // FZ >= 0

		// Friction constraints
		AF.block<4,3>(5,0) <<
				1, 0, MU,                  // MU*FZ + FX >= 0
				-1, 0, MU,                 // MU*FZ - FX >= 0
				0, 1, MU,                  // MU*FZ + FY >= 0
				0, -1, MU;                 // MU*FZ - FY >= 0

		AF = AF * Rot;
		break;
	case SSR:
		DX_toe = sup_box[RIGHT][0];
		DX_heel = sup_box[RIGHT][1];
		DY_left = sup_box[RIGHT][2];
		DY_right = sup_box[RIGHT][3];
		// COP constraints
		AF.block<4,6>(0,0) <<
				0, 0, DX_toe, 0, 1, 0,     // DX_t*FZ + MY >= 0
				0, 0, -DX_heel, -0, -1, 0,  // -DX_h*FZ - MY >= 0
				0, 0, DY_left, -1, 0, 0,   // DY_l*FZ - MX >= 0
				0, 0, -DY_right, 1, -0, 0;  // -DY_r*FZ + MX >= 0
		AF(4,2)=1;                     // FZ >= 0

		// Friction constraints
		AF.block<4,3>(5,0) <<
				1, 0, MU,                  // MU*FZ + FX >= 0
				-1, 0, MU,                 // MU*FZ - FX >= 0
				0, 1, MU,                  // MU*FZ + FY >= 0
				0, -1, MU;                 // MU*FZ - FY >= 0

		AF = AF * Rot;
		break;
	default: assert(0); // BUG!
	}

	// torque limit
	// I*tau - lower >= 0, -I*tau + upper >= 0
	Eigen::Matrix<double,2*PELV_N_JOINTS,PELV_N_JOINTS> TauLim;
	TauLim.block<PELV_N_JOINTS,PELV_N_JOINTS>(0,0) = Eigen::Matrix<double,PELV_N_JOINTS,PELV_N_JOINTS>::Identity();
	TauLim.block<PELV_N_JOINTS,PELV_N_JOINTS>(PELV_N_JOINTS,0) = -Eigen::Matrix<double,PELV_N_JOINTS,PELV_N_JOINTS>::Identity();

	Eigen::Matrix<double,2*PELV_N_JOINTS,1> tauLim;
	for (int i = 0; i < PELV_N_JOINTS; i++)
		Transmission::getTorqueLimit(i, rs.joints[i], tauLim.data()+i, tauLim.data()+i+PELV_N_JOINTS);
	tauLim.segment<PELV_N_JOINTS>(0) *= -1;

	//inflate MBY
	tauLim[A_BACK_MBY] *= 5.0;
	tauLim[A_BACK_MBY+PELV_N_JOINTS] *= 5.0;

	//tauLim.segment<PELV_N_JOINTS>(0) = -Eigen::Map<const Eigen::Matrix<double,PELV_N_JOINTS,1> > (RobotState::torqueLimit[0]);
	//tauLim.segment<PELV_N_JOINTS>(PELV_N_JOINTS) = Eigen::Map<const Eigen::Matrix<double,PELV_N_JOINTS,1> > (RobotState::torqueLimit[1]);

	//tauLim *= 10;

	//=======
	/*
  min 0.5 * x G x + g0 x
  s.t.
    CE^T x + ce0 = 0
    CI^T x + ci0 >= 0 
	 */

	assert(rowIdx <= MAX_ROWS);

	//std::ofstream out;

	Eigen::Matrix<double, 8, 6> CIhands;
	Eigen::Matrix<double, 8, 1> ci0hands;
	ci0hands.setZero();


	switch(handFricMode) {
	case 0:
		CIhands(0,0) = 1;		CIhands(0,2) = HAND_MU;
		CIhands(1,2) = 1;	//z >0
		CIhands(2,0) = HAND_MU;	CIhands(2,1) = 1;
		CIhands(4,3) = 1;		CIhands(4,5) = HAND_MU;
		CIhands(5,5) = 1;	//z >0
		CIhands(7,3) = HAND_MU;	CIhands(7,4) = -1;
		break;
	case 1:
		CIhands(0,2) = -1;	CIhands(0,0) = HAND_MU;
		CIhands(1,2) = 1;	CIhands(1,0) = HAND_MU;
		CIhands(2,1) = -1;	CIhands(2,0) = HAND_MU;
		CIhands(3,1) = 1;	CIhands(3,0) = HAND_MU;
		CIhands(4,5) = -1;	CIhands(4,3) = HAND_MU;
		CIhands(5,5) = 1;	CIhands(5,3) = HAND_MU;
		CIhands(6,4) = -1;	CIhands(6,3) = HAND_MU;
		CIhands(7,4) = 1;	CIhands(7,3) = HAND_MU;
		break;
	default:
		fprintf(stderr, "bad handFricMode\n");
		break;
	}



	Eigen::Map<Eigen::VectorXd> X(_X.data(),nVars);
	switch (cmd.contactState) {
	case DSl:
	case DSr:
	case DSc: {
		const int N_VARS = PELV_N_SDFAST_U + PELV_N_JOINTS + 12 + 6;
		const int N_EQ_CON = PELV_N_SDFAST_U;
		const int N_INEQ_CON = 9*12/6 + PELV_N_JOINTS*2 + 8;

		Eigen::Matrix<double,N_VARS,N_VARS> G =	A.topRows(rowIdx).transpose()*A.topRows(rowIdx);
		Eigen::Matrix<double,N_VARS,1> g0 = -A.topRows(rowIdx).transpose()*b.topRows(rowIdx);
		Eigen::Matrix<double,N_VARS,N_EQ_CON> CE = AD.transpose();
		Eigen::Matrix<double,N_EQ_CON,1> ce0 = -bD;


		Eigen::Matrix<double,N_INEQ_CON,N_VARS> CI;
		CI.setZero();
		CI.block<2*PELV_N_JOINTS,PELV_N_JOINTS>(0,torqueStart) = TauLim;
		CI.block<9*12/6,12>(2*PELV_N_JOINTS,forceStart) = AF;

		Eigen::Matrix<double,N_INEQ_CON,1> ci0;
		ci0.setZero();
		ci0.segment<PELV_N_JOINTS*2>(0) = tauLim;

		//hand friction constraints
		CI.block<8,6>(9*12/6 + PELV_N_JOINTS*2, hfStart) = CIhands;
		ci0.segment<8>(9*12/6 + PELV_N_JOINTS*2) = ci0hands;

		//turn of inequality constraints
		//ci0.setZero();
		//CI.setZero();

		Eigen::Matrix<double,N_VARS,1> x;
		QPval = Eigen::solve_quadprog<N_VARS,N_EQ_CON,N_INEQ_CON>(G, g0, CE, ce0, CI.transpose(), ci0, x);
		X.topRows<N_VARS>() = x;

		for (int i = 0; i < N_VARS; i++)
			if(isnan(x[i])) {
				fprintf(stderr, "NaN's in ID\n");
				logPoint->saveData();
				logPoint->writeToMRDPLOT();
				exit(-1);
			}
		break;
	}
	case SSL:
	case SSR: {

		const int N_VARS = PELV_N_SDFAST_U + PELV_N_JOINTS + 6 + 6;
		const int N_EQ_CON = PELV_N_SDFAST_U;
		const int N_INEQ_CON = 9*6/6 + 2*PELV_N_JOINTS + 8;

		Eigen::Matrix<double,N_VARS,N_VARS> G =	A.topRows(rowIdx).transpose()*A.topRows(rowIdx);
		Eigen::Matrix<double,N_VARS,1> g0 =	-A.topRows(rowIdx).transpose()*b.topRows(rowIdx);
		Eigen::Matrix<double,N_VARS,N_EQ_CON> CE = AD.transpose();
		Eigen::Matrix<double,N_EQ_CON,1> ce0 = -bD;

		Eigen::Matrix<double,N_INEQ_CON,N_VARS> CI;
		CI.setZero();
		CI.block<2*PELV_N_JOINTS,PELV_N_JOINTS>(0,torqueStart) = TauLim;
		CI.block<9*6/6,6>(2*PELV_N_JOINTS,forceStart) = AF;

		Eigen::Matrix<double,N_INEQ_CON,1> ci0;
		ci0.setZero();
		ci0.segment<PELV_N_JOINTS*2>(0) = tauLim;


		//hand friction constraints
		CI.block<8,6>(9*6/6 + PELV_N_JOINTS*2, hfStart) = CIhands;
		ci0.segment<8>(9*6/6 + PELV_N_JOINTS*2) = ci0hands;

		Eigen::Matrix<double,N_VARS,1> x;
		QPval = Eigen::solve_quadprog<N_VARS,N_EQ_CON,N_INEQ_CON>(G, g0, CE, ce0, CI.transpose(), ci0, x);
		X.topRows<N_VARS>() = x;

		for (int i = 0; i < N_VARS; i++)
			assert(!isnan(x[i]));
		break;
	}
	default: assert(0);
	}
	if(QPval == std::numeric_limits<double>::infinity())	Logger::setErrEW(2);


	Eigen::Matrix<double,PELV_N_SDFAST_U,1> id_acc(id_acc = X.topRows(PELV_N_SDFAST_U));


	//for recording
	for(int i = 0; i < PELV_N_SDFAST_U; i++)	Qdd[i] = id_acc(i,0);
	Eigen::MatrixXd cdd = rs.Jc.transpose()*id_acc;
	Eigen::MatrixXd pdd = rs.Jpelvis.block<PELV_N_SDFAST_U,3>(0,3).transpose()*id_acc + rs.Jpelvisd.block<PELV_N_SDFAST_U,3>(0,3).transpose()*qdot * QD_JD_USE_FRAC;
	Eigen::MatrixXd tdd = rs.Jtorso.block<PELV_N_SDFAST_U,3>(0,3).transpose()*id_acc + rs.Jtorsod.block<PELV_N_SDFAST_U,3>(0,3).transpose()*qdot * QD_JD_USE_FRAC;

	for(int i = 0; i < 3; i++) {
		comdd[i] = cdd(i,0);
		pelvdd[i] = pdd(i,0);
		torsodd[i] = tdd(i,0);
	}
	for(int i = 0; i < 2; i++) {
		Eigen::MatrixXd fdd = rs.J[i].transpose()*id_acc;
		for(int j = 0; j < 6; j++)	footdd[i][j] = fdd(j,0);
		Eigen::MatrixXd hdd = rs.Jhand[i].transpose()*id_acc;
		for(int j = 0; j < 6; j++)	handdd[i][j] = hdd(j,0);
	}

	/////////////////////////////////////////////////////////////
	// integrate magic force
	if(cmd.magicTorqueGain != 0) {
		_magicFT.segment<3>(3) -= cmd.magicTorqueGain*id_acc.segment<3>(3);
	}


	//_magicFT.segment<3>(0) -= magicForceGain*id_acc.segment<3>(0);

	if (cmd.magicCopGain != 0) {
		_magicCop[XX] += cmd.magicCopGain * (cmd.cop[XX] - rs.com[XX]);
		_magicCop[YY] += cmd.magicCopGain * (cmd.cop[YY] - rs.com[YY]);
	}

	/////////////////////////////////////////////////////////////

	// save old force
	switch (cmd.contactState) {
	case DSl:
	case DSr:
	case DSc:
		_forceOld = X.segment<12>(forceStart);
		break;
	case SSL:
		_forceOld << X.segment<6>(forceStart), Eigen::Matrix<double,6,1>::Zero();
		break;
	case SSR:
		_forceOld << Eigen::Matrix<double,6,1>::Zero(), X.segment<6>(forceStart);
		break;
	default: assert(0); // BUG!
	}

	cop[0] = (_forceOld(2)*rs.feet[0].w_sensor_pos[0] - _forceOld(4) + _forceOld(8)*rs.feet[1].w_sensor_pos[0] - _forceOld(10))/(_forceOld(2)+_forceOld(8));
	cop[1] = (_forceOld(2)*rs.feet[0].w_sensor_pos[1] + _forceOld(3) + _forceOld(8)*rs.feet[1].w_sensor_pos[1] + _forceOld(9))/(_forceOld(2)+_forceOld(8));

	for(int s = 0; s < 2; s++)		for(int i = 0; i < 3; i++)		handForce[s][i] = X[hfStart+3*s+i];

	// save old torque
	_torqueOld = X.segment<PELV_N_JOINTS>(PELV_N_SDFAST_U);
	if(torque)		for(int i = 0; i < PELV_N_JOINTS; i++) torque[i] = X[i+PELV_N_SDFAST_U];

	return true;
}

double PelvIdCon::getWl() {
	return _forceOld[2]/(_forceOld[2]+_forceOld[8]);
}

void PelvIdCon::getCommand(double *torque) {
	for(int i = 0; i < PELV_N_JOINTS; i++) torque[i] = _torqueOld[i];
}

void PelvIdCon::setCommand(const double *torque, const double *F) {
	for(int i = 0; i < PELV_N_JOINTS; i++)	 _torqueOld[i] = torque[i];
	if(F)	for(int i = 0; i < 12; i++)	 _forceOld[i] = F[i];
}

void PelvIdCon::addToLog(BatchLogger &logger) {
	logPoint = &logger;
	logger.add_datapoint("ID.QPval","-",&QPval);
	logger.add_datapoint("ID.comdd[X]","a",&(comdd[XX]));
	logger.add_datapoint("ID.comdd[Y]","a",&(comdd[YY]));
	logger.add_datapoint("ID.comdd[Z]","a",&(comdd[ZZ]));

	logger.add_datapoint("ID.pelvWd[X]","ra",&(pelvdd[XX]));
	logger.add_datapoint("ID.pelvWd[Y]","ra",&(pelvdd[YY]));
	logger.add_datapoint("ID.pelvWd[Z]","ra",&(pelvdd[ZZ]));

	logger.add_datapoint("ID.torsoWd[X]","ra",&(torsodd[XX]));
	logger.add_datapoint("ID.torsoWd[Y]","ra",&(torsodd[YY]));
	logger.add_datapoint("ID.torsoWd[Z]","ra",&(torsodd[ZZ]));

	logger.add_datapoint("ID.cop[X]","m",&(cop[XX]));
	logger.add_datapoint("ID.cop[Y]","m",&(cop[YY]));

	logger.add_datapoint("ID.ftdd[L][X]","a",&(footdd[LEFT][XX]));
	logger.add_datapoint("ID.ftdd[L][Y]","a",&(footdd[LEFT][YY]));
	logger.add_datapoint("ID.ftdd[L][Z]","a",&(footdd[LEFT][ZZ]));
	logger.add_datapoint("ID.ftdd[R][X]","a",&(footdd[RIGHT][XX]));
	logger.add_datapoint("ID.ftdd[R][Y]","a",&(footdd[RIGHT][YY]));
	logger.add_datapoint("ID.ftdd[R][Z]","a",&(footdd[RIGHT][ZZ]));

	logger.add_datapoint("ID.ftdd[L][AX]","ra",&(footdd[LEFT][XX+3]));
	logger.add_datapoint("ID.ftdd[L][AY]","ra",&(footdd[LEFT][YY+3]));
	logger.add_datapoint("ID.ftdd[L][AZ]","ra",&(footdd[LEFT][ZZ+3]));
	logger.add_datapoint("ID.ftdd[R][AX]","ra",&(footdd[RIGHT][XX+3]));
	logger.add_datapoint("ID.ftdd[R][AY]","ra",&(footdd[RIGHT][YY+3]));
	logger.add_datapoint("ID.ftdd[R][AZ]","ra",&(footdd[RIGHT][ZZ+3]));

	logger.add_datapoint("ID.hadd[L][X]","a",&(handdd[LEFT][XX]));
	logger.add_datapoint("ID.hadd[L][Y]","a",&(handdd[LEFT][YY]));
	logger.add_datapoint("ID.hadd[L][Z]","a",&(handdd[LEFT][ZZ]));
	logger.add_datapoint("ID.hadd[R][X]","a",&(handdd[RIGHT][XX]));
	logger.add_datapoint("ID.hadd[R][Y]","a",&(handdd[RIGHT][YY]));
	logger.add_datapoint("ID.hadd[R][Z]","a",&(handdd[RIGHT][ZZ]));

	logger.add_datapoint("ID.hadd[L][AX]","ra",&(handdd[LEFT][XX+3]));
	logger.add_datapoint("ID.hadd[L][AY]","ra",&(handdd[LEFT][YY+3]));
	logger.add_datapoint("ID.hadd[L][AZ]","ra",&(handdd[LEFT][ZZ+3]));
	logger.add_datapoint("ID.hadd[R][AX]","ra",&(handdd[RIGHT][XX+3]));
	logger.add_datapoint("ID.hadd[R][AY]","ra",&(handdd[RIGHT][YY+3]));
	logger.add_datapoint("ID.hadd[R][AZ]","ra",&(handdd[RIGHT][ZZ+3]));

	logger.add_datapoint("ID.Qdd[X]","r",&(Qdd[0]));
	logger.add_datapoint("ID.Qdd[Y]","r",&(Qdd[1]));
	logger.add_datapoint("ID.Qdd[Z]","r",&(Qdd[2]));
	logger.add_datapoint("ID.Qdd[AX]","r",&(Qdd[3]));
	logger.add_datapoint("ID.Qdd[AY]","r",&(Qdd[4]));
	logger.add_datapoint("ID.Qdd[AZ]","r",&(Qdd[5]));
	logger.add_datapoint("ID.Qdd[back_lbz]","r",&(Qdd[6]));
	logger.add_datapoint("ID.Qdd[back_mby]","r",&(Qdd[7]));
	logger.add_datapoint("ID.Qdd[back_ubx]","r",&(Qdd[8]));
	logger.add_datapoint("ID.Qdd[neck_ay]","r",&(Qdd[9]));
	logger.add_datapoint("ID.Qdd[l_leg_uhz]","r",&(Qdd[10]));
	logger.add_datapoint("ID.Qdd[l_leg_mhx]","r",&(Qdd[11]));
	logger.add_datapoint("ID.Qdd[l_leg_lhy]","r",&(Qdd[12]));
	logger.add_datapoint("ID.Qdd[l_leg_kny]","r",&(Qdd[13]));
	logger.add_datapoint("ID.Qdd[l_leg_uay]","r",&(Qdd[14]));
	logger.add_datapoint("ID.Qdd[l_leg_lax]","r",&(Qdd[15]));
	logger.add_datapoint("ID.Qdd[r_leg_uhz]","r",&(Qdd[16]));
	logger.add_datapoint("ID.Qdd[r_leg_mhx]","r",&(Qdd[17]));
	logger.add_datapoint("ID.Qdd[r_leg_lhy]","r",&(Qdd[18]));
	logger.add_datapoint("ID.Qdd[r_leg_kny]","r",&(Qdd[19]));
	logger.add_datapoint("ID.Qdd[r_leg_uay]","r",&(Qdd[20]));
	logger.add_datapoint("ID.Qdd[r_leg_lax]","r",&(Qdd[21]));
	logger.add_datapoint("ID.Qdd[l_arm_usy]","r",&(Qdd[22]));
	logger.add_datapoint("ID.Qdd[l_arm_shx]","r",&(Qdd[23]));
	logger.add_datapoint("ID.Qdd[l_arm_ely]","r",&(Qdd[24]));
	logger.add_datapoint("ID.Qdd[l_arm_elx]","r",&(Qdd[25]));
	logger.add_datapoint("ID.Qdd[l_arm_uwy]","r",&(Qdd[26]));
	logger.add_datapoint("ID.Qdd[l_arm_mwx]","r",&(Qdd[27]));
	logger.add_datapoint("ID.Qdd[r_arm_usy]","r",&(Qdd[28]));
	logger.add_datapoint("ID.Qdd[r_arm_shx]","r",&(Qdd[29]));
	logger.add_datapoint("ID.Qdd[r_arm_ely]","r",&(Qdd[30]));
	logger.add_datapoint("ID.Qdd[r_arm_elx]","r",&(Qdd[31]));
	logger.add_datapoint("ID.Qdd[r_arm_uwy]","r",&(Qdd[32]));
	logger.add_datapoint("ID.Qdd[r_arm_mwx]","r",&(Qdd[33]));

	logger.add_datapoint("ID.trq[back_lbz]","r",&(_torqueOld[0]));
	logger.add_datapoint("ID.trq[back_mby]","r",&(_torqueOld[1]));
	logger.add_datapoint("ID.trq[back_ubx]","r",&(_torqueOld[2]));
	logger.add_datapoint("ID.trq[neck_ay]","r",&(_torqueOld[3]));
	logger.add_datapoint("ID.trq[l_leg_uhz]","r",&(_torqueOld[4]));
	logger.add_datapoint("ID.trq[l_leg_mhx]","r",&(_torqueOld[5]));
	logger.add_datapoint("ID.trq[l_leg_lhy]","r",&(_torqueOld[6]));
	logger.add_datapoint("ID.trq[l_leg_kny]","r",&(_torqueOld[7]));
	logger.add_datapoint("ID.trq[l_leg_uay]","r",&(_torqueOld[8]));
	logger.add_datapoint("ID.trq[l_leg_lax]","r",&(_torqueOld[9]));
	logger.add_datapoint("ID.trq[r_leg_uhz]","r",&(_torqueOld[10]));
	logger.add_datapoint("ID.trq[r_leg_mhx]","r",&(_torqueOld[11]));
	logger.add_datapoint("ID.trq[r_leg_lhy]","r",&(_torqueOld[12]));
	logger.add_datapoint("ID.trq[r_leg_kny]","r",&(_torqueOld[13]));
	logger.add_datapoint("ID.trq[r_leg_uay]","r",&(_torqueOld[14]));
	logger.add_datapoint("ID.trq[r_leg_lax]","r",&(_torqueOld[15]));
	logger.add_datapoint("ID.trq[l_arm_usy]","r",&(_torqueOld[16]));
	logger.add_datapoint("ID.trq[l_arm_shx]","r",&(_torqueOld[17]));
	logger.add_datapoint("ID.trq[l_arm_ely]","r",&(_torqueOld[18]));
	logger.add_datapoint("ID.trq[l_arm_elx]","r",&(_torqueOld[19]));
	logger.add_datapoint("ID.trq[l_arm_uwy]","r",&(_torqueOld[20]));
	logger.add_datapoint("ID.trq[l_arm_mwx]","r",&(_torqueOld[21]));
	logger.add_datapoint("ID.trq[r_arm_usy]","r",&(_torqueOld[22]));
	logger.add_datapoint("ID.trq[r_arm_shx]","r",&(_torqueOld[23]));
	logger.add_datapoint("ID.trq[r_arm_ely]","r",&(_torqueOld[24]));
	logger.add_datapoint("ID.trq[r_arm_elx]","r",&(_torqueOld[25]));
	logger.add_datapoint("ID.trq[r_arm_uwy]","r",&(_torqueOld[26]));
	logger.add_datapoint("ID.trq[r_arm_mwx]","r",&(_torqueOld[27]));

	logger.add_datapoint("ID.F[L][X]","N",&(_forceOld[0]));
	logger.add_datapoint("ID.F[L][Y]","N",&(_forceOld[1]));
	logger.add_datapoint("ID.F[L][Z]","N",&(_forceOld[2]));
	logger.add_datapoint("ID.F[R][X]","N",&(_forceOld[6]));
	logger.add_datapoint("ID.F[R][Y]","N",&(_forceOld[7]));
	logger.add_datapoint("ID.F[R][Z]","N",&(_forceOld[8]));
	logger.add_datapoint("ID.M[L][X]","Nm",&(_forceOld[3]));
	logger.add_datapoint("ID.M[L][Y]","Nm",&(_forceOld[4]));
	logger.add_datapoint("ID.M[L][Z]","Nm",&(_forceOld[5]));
	logger.add_datapoint("ID.M[R][X]","Nm",&(_forceOld[9]));
	logger.add_datapoint("ID.M[R][Y]","Nm",&(_forceOld[10]));
	logger.add_datapoint("ID.M[R][Z]","Nm",&(_forceOld[11]));

	logger.add_datapoint("ID.HF[L][X]","N",&(handForce[LEFT][XX]));
	logger.add_datapoint("ID.HF[L][Y]","N",&(handForce[LEFT][YY]));
	logger.add_datapoint("ID.HF[L][Z]","N",&(handForce[LEFT][ZZ]));
	logger.add_datapoint("ID.HF[R][X]","N",&(handForce[RIGHT][XX]));
	logger.add_datapoint("ID.HF[R][Y]","N",&(handForce[RIGHT][YY]));
	logger.add_datapoint("ID.HF[R][Z]","N",&(handForce[RIGHT][ZZ]));

}

