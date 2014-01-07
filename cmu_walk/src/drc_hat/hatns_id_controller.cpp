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

#include "id_controller.hpp"
#include "eiquadprog.hpp"
#include "drc_hat_ns_defines.h"
#include "Utils.hpp"
#include "Transmission.h"

bool HatNSIdCon::ID(const RobotState &rs, const IDcmd &cmd, Command &output)
{ 
  if (rs.getType() != _type)
    return false;
  
  double sup_box[2][4];
	rs.feet[LEFT].getBoundingBox(Foot::RefSensor, sup_box[LEFT]);
	rs.feet[RIGHT].getBoundingBox(Foot::RefSensor, sup_box[RIGHT]); 
  
  static Eigen::Quaterniond footq_h[LR] = {Eigen::Quaterniond::Identity(), Eigen::Quaterniond::Identity()}; 
  const double e = 0.9;

  int nForces = 0;
  
  // 2 * [Fxyz,Txyz]
  switch (rs.contactState) {
    case DSl:
    case DSr:
    case DSc: nForces = 12; break;
    case SSL:    
    case SSR: nForces = 6;  break;
    default:     
      assert(0);
  }

  // nVars is [u_dot, tau, 2*contact_f]'
  int nVars = HATNS_N_SDFAST_U + HATNS_N_JOINTS + nForces;
  int nEqCon = HATNS_N_SDFAST_U;
  int nIneqCon = 9*nForces/6;
  const int forceStart = HATNS_N_SDFAST_U + HATNS_N_JOINTS;
  const int torqueStart = HATNS_N_SDFAST_U;
  
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
  
  switch (rs.contactState) {
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
  
  // fot jdot
  Eigen::Matrix<double,HATNS_N_SDFAST_U,1> qdot;
  qdot << Eigen::Map<const Eigen::Matrix<double,6,1> >(rs.rootd),
          Eigen::Map<const Eigen::Matrix<double,HATNS_N_JOINTS,1> >(rs.jointsd);
  qdot *= QD_JD_USE_FRAC;
  double z = (rs.feet[LEFT].w_sensor_pos[ZZ] + rs.feet[RIGHT].w_sensor_pos[ZZ]) / 2.;
   
  int rowIdx = 0;
  
  // rootdd 
  /***************************
   * AD = [I, 0, 0, 0];
   * bD = kp(q* - q) + kd(qd* - qd);
   ****************************/
  /*
  b.segment<3>(rowIdx) = Eigen::Map< const Eigen::Matrix<double,3,1> >(cmd.rootdd);
  b.segment<3>(rowIdx+3) = Eigen::Map< const Eigen::Matrix<double,3,1> >(cmd.root_b_wd);

  for (int i = 0; i < 6; i++) {
    // unspecified
    if (b(rowIdx+i) == std::numeric_limits<double>::infinity()) {
      A(rowIdx+i,i) = 2e-1;
      b(rowIdx+i) = 0; //2e-1*(-10*rs.rootd[i]);
    }
    else {
      A(rowIdx+i,i) = 1;
      b(rowIdx+i) *= 1;
    } 
    if (i < 3) {
      A(rowIdx+i,i) *= ROOTDD_REG_WEIGHT;
      b(rowIdx+i) *= ROOTDD_REG_WEIGHT;
    }
    else {
      A(rowIdx+i,i) *= ANGDD_P_WEIGHT;
      b(rowIdx+i) *= ANGDD_P_WEIGHT;
    }
  }
  assert(!(b.segment<6>(rowIdx).array() == std::numeric_limits<double>::infinity()).any());
  rowIdx += 6;
  */
  for (int i = 0; i < 3; i++) {
    A(rowIdx+i,i) = 2e-1;
    b(rowIdx+i) = 0;
  }
  A.block(rowIdx,0,3,3) *= ROOTDD_REG_WEIGHT;
  b.segment<3>(rowIdx) *= ROOTDD_REG_WEIGHT;
  rowIdx += 3;
  
  b.segment<3>(rowIdx) = Eigen::Map<const Eigen::Matrix<double,3,1> >(cmd.pelvWd);
  A.block<3,HATNS_N_SDFAST_U>(rowIdx,0) = rs.Jpelvis.block<HATNS_N_SDFAST_U,3>(0,3).transpose() * ANGDD_P_WEIGHT;
  b.segment<3>(rowIdx) += -rs.Jpelvisd.block<HATNS_N_SDFAST_U,3>(0,3).transpose()*qdot;
  b.segment<3>(rowIdx) *= ANGDD_P_WEIGHT;
  rowIdx += 3; 

  // jointsdd
  /***************************
   * AD = [I, 0, 0, 0];
   * bD = kp(q* - q) + kd(qd* - qd);
   ****************************/
  Eigen::Matrix<double,HATNS_N_JOINTS,1> tmpjdd;
  HatNSRobotState::pelv2hatNS(cmd.jointsdd, tmpjdd.data());
  b.segment<HATNS_N_JOINTS>(rowIdx) = tmpjdd;
  int pelvIdx;
  for (int i = 0; i < HATNS_N_JOINTS; i++) {
    if (b(rowIdx+i) == std::numeric_limits<double>::infinity()) {
      A(rowIdx+i,6+i) = 2e-1;
      b(rowIdx+i) = 0;
      
      //pelvIdx = HatNSRobotState::hatNSIdx2pelvIdx(i);
      //b(rowIdx+i) = getJointLimitAcc(pelvIdx, rs.joints[pelvIdx]);
    }
    else {
      A(rowIdx+i,6+i) = 1;
    }
  }
  //printf("===============================\n");
  
  A.block(rowIdx,6,HATNS_N_JOINTS,HATNS_N_JOINTS) *= QDD_REG_WEIGHT;
  b.segment<HATNS_N_JOINTS>(rowIdx) *= QDD_REG_WEIGHT;
  assert(!(b.segment<HATNS_N_JOINTS>(rowIdx).array() == std::numeric_limits<double>::infinity()).any());
  rowIdx += HATNS_N_JOINTS;
   
  //=== Desired COM qdd
	/***************************
	 * AcomQdd = [Jcom, 0, 0, 0]
	 * bcomQdd = [comdd + -Jd * qd];
   ****************************/
  b.segment<3>(rowIdx) = 
      Eigen::Map<const Eigen::Matrix<double,3,1> >(cmd.comdd);
  if (!((b.segment<3>(rowIdx).array() == 
       std::numeric_limits<double>::infinity()).any()) && COMDD_XY_WEIGHT != 0) 
  {
    A.block<3,HATNS_N_SDFAST_U>(rowIdx,0) = rs.Jc.transpose();
    b.segment<3>(rowIdx) += -rs.Jcd.transpose()*qdot;
    // hack pelvis z tracking
    if (cmd.rootMode[ZZ]) {
      A.block<1,HATNS_N_SDFAST_U>(rowIdx+ZZ,0).setZero();
      A(rowIdx+ZZ, ZZ) = 1;
      b(rowIdx+ZZ) = cmd.comdd[ZZ]; // J is constant
    }
   
    A.block<3,HATNS_N_SDFAST_U>(rowIdx,0) *= COMDD_XY_WEIGHT;
    b.segment<3>(rowIdx) *= COMDD_XY_WEIGHT;
    assert(!(b.segment<3>(rowIdx).array() == std::numeric_limits<double>::infinity()).any());
    rowIdx += 3; 
    /*
    A.block<3,HATNS_N_SDFAST_U>(rowIdx,0) = rs.Jc.transpose() * COMDD_XY_WEIGHT;
    b.segment<3>(rowIdx) += -rs.Jcd.transpose()*qdot;
    b(rowIdx+XX) -= _magicCop[XX] * GRAVITY / z;
    b(rowIdx+YY) -= _magicCop[YY] * GRAVITY / z;
    */
  }
  else
    b.segment<3>(rowIdx).setZero();
  
  //=== Desired utorso orientation
	/***************************
	 * AcomQdd = [Jtorso, 0, 0, 0]
	 * bcomQdd = [torsodd + -Jd * qd];
   ****************************/
  /*
  b.segment<3>(rowIdx) = 
      Eigen::Map<const Eigen::Matrix<double,3,1> >(cmd.torsoWd);
  if (!((b.segment<3>(rowIdx).array() == 
       std::numeric_limits<double>::infinity()).any()) && ANGDD_T_WEIGHT != 0) 
  {
    A.block<3,HATNS_N_SDFAST_U>(rowIdx,0) = rs.Jtorso.block<HATNS_N_SDFAST_U,3>(0,3).transpose() * ANGDD_T_WEIGHT;
    b.segment<3>(rowIdx) += -rs.Jtorsod.block<HATNS_N_SDFAST_U,3>(0,3).transpose()*qdot;
    b.segment<3>(rowIdx) *= ANGDD_T_WEIGHT;
    assert(!(b.segment<3>(rowIdx).array() == std::numeric_limits<double>::infinity()).any());
    rowIdx += 3; 
  }
  else
    b.segment<3>(rowIdx).setZero();
  */
  
  //=== feet accelerations
  /***************************
   * AC = [JL,     0,     0;
   *       JR,     0,     0]
   * bC = [footdd[LEFT] - Jd*qd;
   *       footdd[RIGHT] - Jd*qd]
   ****************************/
  // left 
  b.segment<6>(rowIdx) = 
      Eigen::Map<const Eigen::Matrix<double,6,1> >(cmd.footdd[LEFT]);
  if (!((b.segment<6>(rowIdx).array() == 
       std::numeric_limits<double>::infinity()).any())) 
  {
    A.block<6,HATNS_N_SDFAST_U>(rowIdx,0) = rs.J[LEFT].transpose() * FOOTDD_WEIGHT;
    b.segment<6>(rowIdx) += -rs.Jd[LEFT].transpose()*qdot;
    b.segment<6>(rowIdx) *= FOOTDD_WEIGHT;
    // stance
    if (isDS(rs.contactState) || rs.contactState == SSL) {
      A.block<6,HATNS_N_SDFAST_U>(rowIdx,0) *= 5e1;
      b.segment<6>(rowIdx) *= 5e1;
    }
    assert(!(b.segment<6>(rowIdx).array() == std::numeric_limits<double>::infinity()).any());
    rowIdx += 6;
  }
  else 
    b.segment<6>(rowIdx).setZero();

  // right
  b.segment<6>(rowIdx) = 
      Eigen::Map<const Eigen::Matrix<double,6,1> >(cmd.footdd[RIGHT]);
  if (!((b.segment<6>(rowIdx).array() == 
       std::numeric_limits<double>::infinity()).any())) 
  {
    A.block<6,HATNS_N_SDFAST_U>(rowIdx,0) = rs.J[RIGHT].transpose() * FOOTDD_WEIGHT;
    b.segment<6>(rowIdx) += -rs.Jd[RIGHT].transpose()*qdot;
    b.segment<6>(rowIdx) *= FOOTDD_WEIGHT;
    // stance
    if (isDS(rs.contactState) || rs.contactState == SSR) {
      A.block<6,HATNS_N_SDFAST_U>(rowIdx,0) *= 5e1;
      b.segment<6>(rowIdx) *= 5e1;
    }
    assert(!(b.segment<6>(rowIdx).array() == std::numeric_limits<double>::infinity()).any());
    rowIdx += 6;
  }
  else 
    b.segment<6>(rowIdx).setZero();
 
  // ======= minimize torques
   /***************************
   * AT = [0, I, 0, 0]
   * bT = 0;
   ****************************/
  A.block<HATNS_N_JOINTS,HATNS_N_JOINTS>(rowIdx,torqueStart).setIdentity();

  A.block<HATNS_N_JOINTS,HATNS_N_JOINTS>(rowIdx,torqueStart) *= TAU_REG_WEIGHT;
  b.segment<HATNS_N_JOINTS>(rowIdx) *= TAU_REG_WEIGHT;
  assert(!(b.segment<HATNS_N_JOINTS>(rowIdx).array() == std::numeric_limits<double>::infinity()).any());
  rowIdx += HATNS_N_JOINTS;
 
  //======= minimize change in torques
   /***************************
   * AprevTrq = [0, I, 0, 0, 0]
   * bprevTrq = [_torqueOld];
   ****************************/
  if (DTAU_REG_WEIGHT != 0) {
  //if (DTAU_REG_WEIGHT != 0 && !_torqueOld.isZero()) {
    A.block<HATNS_N_JOINTS,HATNS_N_JOINTS>(rowIdx,torqueStart).setIdentity();
    b.segment<HATNS_N_JOINTS>(rowIdx) = _torqueOld.segment<HATNS_N_JOINTS>(0);
  
    A.block<HATNS_N_JOINTS,HATNS_N_JOINTS>(rowIdx,torqueStart) *= DTAU_REG_WEIGHT;
    b.segment<HATNS_N_JOINTS>(rowIdx) *= DTAU_REG_WEIGHT;
    assert(!(b.segment<HATNS_N_JOINTS>(rowIdx).array() == std::numeric_limits<double>::infinity()).any());
    rowIdx += HATNS_N_JOINTS;
  }
   
  //======= minimize contact forces
   /***************************
   * AcontactF = [0, 0, I, 0;
   *              0, 0, 0, I]
   * bcontactF = [0;0];
   ****************************/
  if (F_REG_WEIGHT != 0) {
    A.block(rowIdx,forceStart,nForces,nForces).setIdentity();
    switch (rs.contactState) {
      case DSl:
      case DSr:
      case DSc:
        if (!isinf(cmd.comdd[ZZ])) {
          b(rowIdx+ZZ) = cmd.wl*rs.m*(GRAVITY+cmd.comdd[ZZ]);
          b(rowIdx+ZZ+6) = (1-cmd.wl)*rs.m*(GRAVITY+cmd.comdd[ZZ]);
        }
        else {
          b(rowIdx+ZZ) = cmd.wl*rs.m*(GRAVITY);
          b(rowIdx+ZZ+6) = (1-cmd.wl)*rs.m*(GRAVITY);
        }
        // yaw
        A(rowIdx+5, forceStart+5) *= 1. / F_REG_WEIGHT;
        A(rowIdx+11, forceStart+11) *= 1. / F_REG_WEIGHT;
        break;
      case SSL:
      case SSR:
        if (!isinf(cmd.comdd[ZZ]))
          b(rowIdx+ZZ) = rs.m*(GRAVITY+cmd.comdd[ZZ]);
        else
          b(rowIdx+ZZ) = rs.m*(GRAVITY);
        // yaw
        A(rowIdx+5, forceStart+5) *= 1e-2 / F_REG_WEIGHT;
        break;
      default: assert(0); // BUG!
    }

    A.block(rowIdx,forceStart,nForces,nForces) *= F_REG_WEIGHT;
    b.segment(rowIdx,nForces) *= F_REG_WEIGHT;
    assert(!(b.segment(rowIdx,nForces).array() == std::numeric_limits<double>::infinity()).any());
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
    switch (rs.contactState) {
      case DSl:
      case DSr:
      case DSc:
      /*
        if (_forceOld.segment<6>(0).isZero())
          A.block(rowIdx,forceStart,6,6).setZero();
        if (_forceOld.segment<6>(6).isZero()) 
          A.block(rowIdx+6,forceStart+6,6,6).setZero();
        */
        b.segment<12>(rowIdx) = _forceOld;
        break;
      case SSL:
        //if (_forceOld.segment<6>(0).isZero())
        //  A.block(rowIdx,forceStart,6,6).setZero();
        b.segment<6>(rowIdx) = _forceOld.segment<6>(0);
        break;
      case SSR:
        //if (_forceOld.segment<6>(6).isZero()) 
        //  A.block(rowIdx+6,forceStart+6,6,6).setZero();
        b.segment<6>(rowIdx) = _forceOld.segment<6>(6);
        break;
      default: assert(0); // BUG!
    }

    A.block(rowIdx,forceStart,nForces,nForces) *= DF_REG_WEIGHT;
    b.segment(rowIdx,nForces) *= DF_REG_WEIGHT;
    assert(!(b.segment(rowIdx,nForces).array() == std::numeric_limits<double>::infinity()).any());
    
    rowIdx += nForces;
  }

  //=== Desired CoP
  /**********************************************
  t_y + p_x_d*Fz = 0  cop x // geometric center 
  t_x + p_y_d*Fz = 0  cop y // geometric center
  right
  ***********************************************/
  // all in foot local frame
  // copx = -My / Fz  => Fz*copx + My = 0
  // copy = Mx / Fz   => Fz*copy - Mx = 0
  if (FOOT_COP_WEIGHT != 0) {
    switch (rs.contactState) {
      case DSl:
      case DSr:
      case DSc:
        A.block<2,6>(rowIdx,forceStart) << 
          0, 0, cmd.b_cop[LEFT][XX], 0, 1, 0, 
          0, 0, 1*cmd.b_cop[LEFT][YY], -1, 0, 0;
        A.block<2,6>(rowIdx+2,forceStart+6) << 
          0, 0, cmd.b_cop[RIGHT][XX], 0, 1, 0,
          0, 0, 1*cmd.b_cop[RIGHT][YY], -1, 0, 0;

        A.block<4,12>(rowIdx,forceStart) = A.block<4,12>(rowIdx,forceStart) * Rot;
        break;
      case SSL:
        A.block<2,6>(rowIdx,forceStart) << 
          0, 0, cmd.b_cop[LEFT][XX], 0, 1, 0, 
          0, 0, 1*cmd.b_cop[LEFT][YY], -1, 0, 0;      
        A.block<2,6>(rowIdx,forceStart) = A.block<2,6>(rowIdx,forceStart) * Rot;
        break;
      case SSR:
        A.block<2,6>(rowIdx,forceStart) << 
          0, 0, cmd.b_cop[RIGHT][XX], 0, 1, 0,
          0, 0, 1*cmd.b_cop[RIGHT][YY], -1, 0, 0;
        A.block<2,6>(rowIdx,forceStart) = A.block<2,6>(rowIdx,forceStart) * Rot;
        break;
      default: assert(0);
    }

    A.block(rowIdx,forceStart,2*nForces/6,nForces) *= FOOT_COP_WEIGHT;
    b.segment(rowIdx,2*nForces/6) *= FOOT_COP_WEIGHT;
    assert(!(b.segment(rowIdx,2*nForces/6).array() == std::numeric_limits<double>::infinity()).any());
    rowIdx += 2*nForces/6;
  }

  //=== weight distribution
  /***************************
   * Fzl/(Fzl+Fzr) = w
   ****************************/
  if (isDS(rs.contactState)) {
    A(rowIdx,forceStart+2) = 1.0 - cmd.wl;
    A(rowIdx,forceStart+6+2) = -cmd.wl;
    
    A.block(rowIdx,forceStart,1,nForces) *= WL_WEIGHT;
    b.segment<1>(rowIdx) *= WL_WEIGHT;
    assert(!(b.segment<1>(rowIdx).array() == std::numeric_limits<double>::infinity()).any());
    rowIdx += 1;
  }
  
  //=== Sum of forces on COM
  /***************************
   * AcomF = [0, 0, [I 0], [I 0]]
   * bcomF = [comdd + mg + task_force];
   ****************************/
  b.segment<3>(rowIdx) = 
      Eigen::Map<const Eigen::Matrix<double,3,1> >(cmd.comdd);
  if (!((b.segment<3>(rowIdx).array() == 
       std::numeric_limits<double>::infinity()).any()) && COM_F_WEIGHT != 0) {
    switch (rs.contactState) {
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
    b.segment<3>(rowIdx) = Eigen::Map<const Eigen::Vector3d>(cmd.comdd);
    b(rowIdx+XX) -= _magicCop[XX]*GRAVITY / (rs.com[ZZ]-z);
    b(rowIdx+YY) -= _magicCop[YY]*GRAVITY / (rs.com[ZZ]-z);
    b(rowIdx+ZZ) += GRAVITY;
    b.segment<3>(rowIdx) *= rs.m;

    A.block(rowIdx,forceStart,3,nForces) *= COM_F_WEIGHT;
    b.segment<3>(rowIdx) *= COM_F_WEIGHT;
    assert(!(b.segment<3>(rowIdx).array() == std::numeric_limits<double>::infinity()).any());
    rowIdx += 3;
  }
  else 
    b.segment<3>(rowIdx).setZero();
       
  //=== Sum of torques on COM
  /***************************
   * AcomTrq = [0, 0,[ML I],[MR I]]
   *
   * ML = (PL - COM)x
   * MR = (PR - COM)x
   * bcomTrq = [0];
   ****************************/
  b.segment<3>(rowIdx) = 
      Eigen::Map<const Eigen::Matrix<double,3,1> >(cmd.rootdd+3);
  if (!((b.segment<3>(rowIdx).array() == 
          std::numeric_limits<double>::infinity()).any()) && COM_TAU_WEIGHT != 0) {
    const Eigen::Matrix3d & ML = makeCrossMatrix(
        Eigen::Map<const Eigen::Vector3d>(rs.feet[LEFT].w_sensor_pos) - 
        Eigen::Map<const Eigen::Vector3d>(rs.com));
    const Eigen::Matrix3d & MR = makeCrossMatrix(
        Eigen::Map<const Eigen::Vector3d>(rs.feet[RIGHT].w_sensor_pos) - 
        Eigen::Map<const Eigen::Vector3d>(rs.com));
    switch (rs.contactState) {
      case DSl:
      case DSr:
      case DSc:
        A.block<3,3>(rowIdx,forceStart) = ML;
        A.block<3,3>(rowIdx,forceStart+3).setIdentity();
        A.block<3,3>(rowIdx,forceStart+6) = MR;
        A.block<3,3>(rowIdx,forceStart+9).setIdentity();
        break;
      case SSL:
        A.block<3,3>(rowIdx,forceStart) = ML;
        A.block<3,3>(rowIdx,forceStart+3).setIdentity();
        break;
      case SSR:
        A.block<3,3>(rowIdx,forceStart) = MR;
        A.block<3,3>(rowIdx,forceStart+3).setIdentity();
        break;
      default: assert(0); // BUG!
    }

    // NOTE:
    //   bcomTrq should equal to whatever com_angular_acceleration comes up with 
    //   in SDModel_common
    b.segment<3>(rowIdx) = rs.I * Eigen::Map<const Eigen::Vector3d>(cmd.rootdd+3);

    A.block(rowIdx,forceStart,3,nForces) *= COM_TAU_WEIGHT;
    b.segment<3>(rowIdx) *= COM_TAU_WEIGHT;
    assert(!(b.segment<3>(rowIdx).array() == std::numeric_limits<double>::infinity()).any());
    rowIdx += 3;
  }
  else 
    b.segment<3>(rowIdx).setZero();
    
  //======= desired center of pressure
	 /***************************
	 * (FLZ+FRZ)*cop_d = FLZ*(PL + COPL) + FRZ*(PR + COPR)
	 * FLZ*(PL + COPL - cop_d) + FRZ*(PR + COPR - cop_d) = 0
	 ****************************/
	if(cmd.cop[XX] != std::numeric_limits<double>::infinity() && 
     cmd.cop[YY] != std::numeric_limits<double>::infinity() && 
     COP_WEIGHT != 0) 
  {
		double llCoP[2] = {cmd.cop[XX] + _magicCop[XX], 
                       cmd.cop[YY] + _magicCop[YY]};

    switch (rs.contactState) {
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
  Eigen::Map<Eigen::MatrixXd> AD(_AD.data(),HATNS_N_SDFAST_U,nVars);
  AD.topLeftCorner<HATNS_N_SDFAST_U,HATNS_N_SDFAST_U>() = rs.M;
  AD.block<HATNS_N_SDFAST_U,HATNS_N_JOINTS>(0,HATNS_N_SDFAST_U) <<
      Eigen::Matrix<double,6,HATNS_N_JOINTS>::Zero(),
      -Eigen::Matrix<double,HATNS_N_JOINTS,HATNS_N_JOINTS>::Identity();
  switch (rs.contactState) {
    case DSl:
    case DSr:
    case DSc: AD.rightCols<12>() << -rs.J[LEFT], -rs.J[RIGHT]; break;
    case SSL: AD.rightCols<6>() = -rs.J[LEFT];  break;
    case SSR: AD.rightCols<6>() = -rs.J[RIGHT]; break;
    default: assert(0); // BUG!
  }
  //Eigen::Map<const Eigen::Matrix<double,HATNS_N_SDFAST_U,1> > bD(rs.nonLin);
  Eigen::Matrix<double,HATNS_N_SDFAST_U,1> bD = rs.nonLin;
  bD.segment<6>(0) += _magicFT;
  
  //=======
  // cop and friction constraints
  double MU = FRICTION_COEF;
  double DX_toe, DX_heel, DY_left, DY_right;

  Eigen::Map<Eigen::MatrixXd> AF(_AF.data(),nIneqCon,nForces);
  AF.setZero(); 
  
  switch (rs.contactState) {
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
  Eigen::Matrix<double,2*HATNS_N_JOINTS,HATNS_N_JOINTS> TauLim;
  TauLim.block<HATNS_N_JOINTS,HATNS_N_JOINTS>(0,0) = Eigen::Matrix<double,HATNS_N_JOINTS,HATNS_N_JOINTS>::Identity();
  TauLim.block<HATNS_N_JOINTS,HATNS_N_JOINTS>(HATNS_N_JOINTS,0) = -Eigen::Matrix<double,HATNS_N_JOINTS,HATNS_N_JOINTS>::Identity();
  
  Eigen::Matrix<double,2*HATNS_N_JOINTS,1> tauLim;
  for (int i = 0; i < HATNS_N_JOINTS; i++) {
    int j = HatNSRobotState::hatNSIdx2pelvIdx(i);
    Transmission::getTorqueLimit(j, rs.joints[j], tauLim.data()+i, tauLim.data()+i+HATNS_N_JOINTS);
  }
  //HatNSRobotState::pelv2hatNS(RobotState::torqueLimit[0], tauLim.segment<HATNS_N_JOINTS>(0).data());
  //HatNSRobotState::pelv2hatNS(RobotState::torqueLimit[1], tauLim.segment<HATNS_N_JOINTS>(HATNS_N_JOINTS).data());
  
  tauLim.segment<HATNS_N_JOINTS>(0) *= -1;


  //=======
  /*
  min 0.5 * x G x + g0 x
  s.t.
    CE^T x + ce0 = 0
    CI^T x + ci0 >= 0 
  */ 

  assert(rowIdx <= MAX_ROWS);
  
  //std::ofstream out;

  Eigen::Map<Eigen::VectorXd> X(_X.data(),nVars);
  switch (rs.contactState) {
    case DSl:
    case DSr:
    case DSc: {
      const int N_VARS = HATNS_N_SDFAST_U + HATNS_N_JOINTS + 12;
      const int N_EQ_CON = HATNS_N_SDFAST_U;
      //const int N_INEQ_CON = 9*12/6;
      const int N_INEQ_CON = 9*12/6 + HATNS_N_JOINTS*2;
      
      Eigen::Matrix<double,N_VARS,N_VARS> G = 
          A.topRows(rowIdx).transpose()*A.topRows(rowIdx);
      Eigen::Matrix<double,N_VARS,1> g0 = 
          -A.topRows(rowIdx).transpose()*b.topRows(rowIdx);
      Eigen::Matrix<double,N_VARS,N_EQ_CON> CE = AD.transpose();
      Eigen::Matrix<double,N_EQ_CON,1> ce0 = -bD;
      //Eigen::Matrix<double,N_VARS,N_INEQ_CON> CI;
      //CI << Eigen::Matrix<double,HATNS_N_SDFAST_U+HATNS_N_JOINTS,N_INEQ_CON>::Zero(),
      //      AF.transpose();
      
      Eigen::Matrix<double,N_INEQ_CON,N_VARS> CI;
      CI.setZero();
      /* 
      CI.block<9*12/6,12>(0,forceStart) = AF;
      */
      CI.block<2*HATNS_N_JOINTS,HATNS_N_JOINTS>(0,torqueStart) = TauLim;
      CI.block<9*12/6,12>(2*HATNS_N_JOINTS,forceStart) = AF;
      
      Eigen::Matrix<double,N_INEQ_CON,1> ci0;
      ci0.setZero();
      ci0.segment<HATNS_N_JOINTS*2>(0) = tauLim;

      Eigen::Matrix<double,N_VARS,1> x;
      Eigen::solve_quadprog<N_VARS,N_EQ_CON,N_INEQ_CON>(G, g0, 
        CE, ce0, CI.transpose(), ci0, x);
      X.topRows<N_VARS>() = x;

      for (int i = 0; i < N_VARS; i++) {
        assert(!isnan(x[i]));
      }
      break;
    }
    case SSL: 
    case SSR: {
      const int N_VARS = HATNS_N_SDFAST_U + HATNS_N_JOINTS + 6;
      const int N_EQ_CON = HATNS_N_SDFAST_U;
      //const int N_INEQ_CON = 9*6/6;
      const int N_INEQ_CON = 9*6/6 + 2*HATNS_N_JOINTS;
      Eigen::Matrix<double,N_VARS,N_VARS> G = 
          A.topRows(rowIdx).transpose()*A.topRows(rowIdx);
      Eigen::Matrix<double,N_VARS,1> g0 = 
          -A.topRows(rowIdx).transpose()*b.topRows(rowIdx);
      Eigen::Matrix<double,N_VARS,N_EQ_CON> CE = AD.transpose();
      Eigen::Matrix<double,N_EQ_CON,1> ce0 = -bD;
      
      //Eigen::Matrix<double,N_VARS,N_INEQ_CON> CI;
      //CI << Eigen::Matrix<double,HATNS_N_SDFAST_U+HATNS_N_JOINTS,N_INEQ_CON>::Zero(),
      //      AF.transpose();
      
      Eigen::Matrix<double,N_INEQ_CON,N_VARS> CI;
      CI.setZero();
      /*
      CI.block<9*6/6,6>(0,forceStart) = AF; 
      */
      CI.block<2*HATNS_N_JOINTS,HATNS_N_JOINTS>(0,torqueStart) = TauLim;
      CI.block<9*6/6,6>(2*HATNS_N_JOINTS,forceStart) = AF; 
      Eigen::Matrix<double,N_INEQ_CON,1> ci0;
      ci0.setZero();
      ci0.segment<HATNS_N_JOINTS*2>(0) = tauLim;

      Eigen::Matrix<double,N_VARS,1> x;
      Eigen::solve_quadprog<N_VARS,N_EQ_CON,N_INEQ_CON>(G, g0, 
        CE, ce0, CI.transpose(), ci0, x);
      X.topRows<N_VARS>() = x;
     
      for (int i = 0; i < N_VARS; i++)
        assert(!isnan(x[i]));
      break;
    }
    default: assert(0);
  }

  // save old torque
  _torqueOld.segment<HATNS_N_JOINTS>(0) = X.segment<HATNS_N_JOINTS>(torqueStart);
   
  // save old force
  switch (rs.contactState) {
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

  
  /////////////////////////////////////////////////////////////
  // process output
  Eigen::Matrix<double,HATNS_N_SDFAST_U,1> id_acc = X.topRows(HATNS_N_SDFAST_U);
  Eigen::Matrix<double,HATNS_N_JOINTS,1> id_trq = X.segment<HATNS_N_JOINTS>(torqueStart);
  Eigen::Matrix<double,12,1> id_grf;

  switch (rs.contactState) {
    case DSl:
    case DSr:
    case DSc:
      id_grf = X.segment<12>(forceStart);
      break;
    case SSL:
      id_grf << X.segment<6>(forceStart), Eigen::Matrix<double,6,1>::Zero();
      break;
    case SSR:
      id_grf << Eigen::Matrix<double,6,1>::Zero(), X.segment<6>(forceStart);
      break;
    default: assert(0); // BUG!
  }
    
  dvec_copy(output.rootdd_d, id_acc.data(), 3);
  HatNSRobotState::hatNS2pelv(id_acc.data()+6, output.jointsdd_d);
  HatNSRobotState::hatNS2pelv(id_trq.data(), output.trq_ff);
  dvec_copy(output.w_grf_d, id_grf.data(), 12);

  // compute body cop
  Eigen::Map <Eigen::Matrix<double,12,1> > b_grf(output.b_grf_d);
  b_grf.setZero();
  switch (rs.contactState) {
    case DSl:
    case DSr:
    case DSc: {
      b_grf = Rot * id_grf;
      break;
    }
    case SSL:
      b_grf.segment<6>(0) = Rot * id_grf.segment<6>(0);
      break;
    case SSR:
      b_grf.segment<6>(6) = Rot * id_grf.segment<6>(6);
      break;
    default: assert(0); // BUG!
  } 
  // just to avoid / by 0
  output.b_cop_d[LEFT][0] = -b_grf[3+1] / (b_grf[2] + 0.01);
  output.b_cop_d[LEFT][1] = b_grf[3+0] / (b_grf[2] + 0.01);
  output.b_cop_d[RIGHT][0] = -b_grf[3+1+6] / (b_grf[2+6] + 0.01);
  output.b_cop_d[RIGHT][1] = b_grf[3+0+6] / (b_grf[2+6] + 0.01);

  if (isDS(rs.contactState)) {
    output.cop_d[XX] = (id_grf(2)*rs.feet[LEFT].w_sensor_pos[XX] - id_grf(4) + id_grf(8)*rs.feet[RIGHT].w_sensor_pos[XX] - id_grf(10))/(id_grf(2)+id_grf(8));
    output.cop_d[YY] = (id_grf(2)*rs.feet[LEFT].w_sensor_pos[YY] + id_grf(3) + id_grf(8)*rs.feet[RIGHT].w_sensor_pos[YY] + id_grf(9))/(id_grf(2)+id_grf(8));
  }
  else if (rs.contactState == SSL) {
    output.cop_d[XX] = (id_grf(2)*rs.feet[LEFT].w_sensor_pos[XX] - id_grf(4))/(id_grf(2));
    output.cop_d[YY] = (id_grf(2)*rs.feet[LEFT].w_sensor_pos[YY] + id_grf(3))/(id_grf(2));
  }
  else if (rs.contactState == SSR) {
    output.cop_d[XX] = (id_grf(2+6)*rs.feet[RIGHT].w_sensor_pos[XX] - id_grf(4+6))/(id_grf(2+6));
    output.cop_d[YY] = (id_grf(2+6)*rs.feet[RIGHT].w_sensor_pos[YY] + id_grf(3+6))/(id_grf(2+6));
  }

  Eigen::Map <Eigen::Vector3d> id_comdd(output.comdd_d);
  id_comdd = rs.Jc.transpose() * id_acc - rs.Jcd.transpose()*qdot;
  
  Eigen::Map <Eigen::Vector3d> id_torsoWd(output.torsoWd_d);
  id_torsoWd = rs.Jtorso.block<HATNS_N_SDFAST_U,3>(0,3).transpose() * id_acc - rs.Jtorsod.block<HATNS_N_SDFAST_U,3>(0,3).transpose()*qdot;
  
  Eigen::Map <Eigen::Vector3d> id_pelvWd(output.pelvWd_d);
  id_pelvWd = rs.Jpelvis.block<HATNS_N_SDFAST_U,3>(0,3).transpose() * id_acc - rs.Jpelvisd.block<HATNS_N_SDFAST_U,3>(0,3).transpose()*qdot;
  
  Eigen::Map <Eigen::Matrix<double,6,1> > id_footdd_l(output.footdd_d[LEFT]);
  id_footdd_l = rs.J[LEFT].transpose() * id_acc - rs.Jd[LEFT].transpose()*qdot;

  Eigen::Map <Eigen::Matrix<double,6,1> > id_footdd_r(output.footdd_d[RIGHT]);
  id_footdd_r = rs.J[RIGHT].transpose() * id_acc - rs.Jd[RIGHT].transpose()*qdot;
   
  /////////////////////////////////////////////////////////////
  // integrate magic force
  if (ID_IS_MAGIC_INT_TORQUE_ON(integrateMagicMask))
    _magicFT.segment<3>(3) -= MAGIC_TRQ_GAIN*X.segment<3>(3);

  if (ID_IS_MAGIC_INT_FORCE_ON(integrateMagicMask))
    _magicFT.segment<3>(0) -= MAGIC_F_GAIN*X.segment<3>(0);

  if (ID_IS_MAGIC_INT_COP_ON(integrateMagicMask)) {
    _magicCop[XX] += MAGIC_COP_GAIN * (cmd.cop[XX] - rs.com[XX]);
    _magicCop[YY] += MAGIC_COP_GAIN * (cmd.cop[YY] - rs.com[YY]);
  }
  dvec_copy(magicFT, _magicFT.data(), 6);
  dvec_copy(magicCop, _magicCop.data(), 2);
  ///////////////////////////////////////////////////////////// 
  
  return true;
}

void HatNSIdCon::addToLog(BatchLogger &logger) {
	logger.add_datapoint("ID.magicF[X]","a", &(_magicFT.data()[XX]));
	logger.add_datapoint("ID.magicF[Y]","a", &(_magicFT.data()[YY]));
	logger.add_datapoint("ID.magicF[Z]","a", &(_magicFT.data()[ZZ]));
	logger.add_datapoint("ID.magicTau[X]","a", &(_magicFT.data()[XX+3]));
	logger.add_datapoint("ID.magicTau[Y]","a", &(_magicFT.data()[YY+3]));
	logger.add_datapoint("ID.magicTau[Z]","a", &(_magicFT.data()[ZZ+3]));

	logger.add_datapoint("ID.magicCop[X]","a", &(_magicCop.data()[XX]));
	logger.add_datapoint("ID.magicCop[Y]","a", &(_magicCop.data()[YY]));
}
