#include "ik_controller.hpp"
#include "drc_pelvis_defines.h"
#include "eiquadprog.hpp"
#include "Utils.hpp"
#include "Eigen_utils.hpp"

bool SFPelvIkCon::IK(const IKcmd &cmd, Command &output)
{
  Eigen::Vector3d Rotvec;

  int rowIdx = 0;

  _A.setZero();
  _b.setZero();
  Eigen::Map<Eigen::MatrixXd> A(_A.data(),MAX_ROWS,PELV_N_SDFAST_U);
  Eigen::Map<Eigen::VectorXd> b(_b.data(),MAX_ROWS);
 
  ////////////////////////////////////////////////////////
  // comd
  // A = Jcom
  // b = comd
  b.segment<3>(rowIdx) = Eigen::Map<const Eigen::Matrix<double,3,1> >(cmd.comd);
  if (!((b.segment<3>(rowIdx).array() == 
       std::numeric_limits<double>::infinity()).any()) && IK_COM_WEIGHT != 0) 
  {
    A.block<3,PELV_N_SDFAST_U>(rowIdx, 0) = ikrs->Jc.transpose();
    for (int i = 0; i < 3; i++)
      b(rowIdx+i) += IK_COM_RATE*(cmd.com[i] - ikrs->com[i]);
    
    // hack pelvis tracking here
    if (cmd.rootMode[ZZ]) {
      A.block<1,PELV_N_SDFAST_U>(rowIdx+ZZ, 0).setZero();
      A(rowIdx+ZZ, ZZ) = 1;

      b(rowIdx+ZZ) = cmd.comd[ZZ] + IK_COM_RATE*(cmd.com[ZZ] - ikrs->root[ZZ]);
    }
    
    A.block<3,PELV_N_SDFAST_U>(rowIdx, 0) *= IK_COM_WEIGHT;
    b.segment<3>(rowIdx) *= IK_COM_WEIGHT;
    assert(!(b.segment<3>(rowIdx).array() == std::numeric_limits<double>::infinity()).any());

    rowIdx += 3;
  }
  else
    b.segment<3>(rowIdx).setZero();

  
  ////////////////////////////////////////////////////////
  // footd
  // A = [Jl; Jr]
  // b = [fld_d; frd_d]
  Eigen::Matrix<double,6,6> Rot_foot[2];
  for (int side = 0; side < LR; side++) {
    Eigen::Quaterniond q = ikrs->feet[side].w_q;
    //Eigen::Quaterniond q = onlyYaw(ikrs->feet[side].w_q);
    Eigen::Matrix3d footRot = q.inverse().normalized().toRotationMatrix();

    Rot_foot[side].setZero();
    Rot_foot[side].block<3,3>(0,0) = footRot;
    Rot_foot[side].block<3,3>(3,3) = footRot; 
  }
    
  for (int side = 0; side < LR; side++) {
    b.segment<6>(rowIdx) = Eigen::Map<const Eigen::Matrix<double,6,1> >(cmd.footd[side]);

    if (cmd.footRef[side] == Foot::RefCenter) {
      A.block<6,PELV_N_SDFAST_U>(rowIdx,0) = ikrs->Jmid[side].transpose();
      for (int i = 0; i < 3; i++) 
        b(rowIdx+i) += IK_POS_RATE*(cmd.foot[side][i] - ikrs->feet[side].w_mid_pos[i]);
    }
    else if (cmd.footRef[side] == Foot::RefToe) {
      A.block<6,PELV_N_SDFAST_U>(rowIdx,0) = ikrs->Jtoe[side].transpose();
      for (int i = 0; i < 3; i++) 
        b(rowIdx+i) += IK_POS_RATE*(cmd.foot[side][i] - ikrs->feet[side].w_toe_pos[i]);
    }
    else if (cmd.footRef[side] == Foot::RefHeel) {
      A.block<6,PELV_N_SDFAST_U>(rowIdx,0) = ikrs->Jheel[side].transpose();
      for (int i = 0; i < 3; i++) 
        b(rowIdx+i) += IK_POS_RATE*(cmd.foot[side][i] - ikrs->feet[side].w_heel_pos[i]);
    }
    else {
      A.block<6,PELV_N_SDFAST_U>(rowIdx,0) = ikrs->J[side].transpose();
      for (int i = 0; i < 3; i++) 
        b(rowIdx+i) += IK_POS_RATE*(cmd.foot[side][i] - ikrs->feet[side].w_sensor_pos[i]);
    }

    Rotvec = quatMinus(ikrs->feet[side].w_q, cmd.footQ[side]);

    for (int i = 3; i < 6; i++)
      b(rowIdx+i) -= IK_POS_RATE*Rotvec[i-3];

    //b.segment<6>(rowIdx) *= IK_FOOT_WEIGHT;

    A.block<6,PELV_N_SDFAST_U>(rowIdx,0) = Rot_foot[side] * A.block<6,PELV_N_SDFAST_U>(rowIdx,0);
    b.segment<6>(rowIdx) = Rot_foot[side] * b.segment<6>(rowIdx);

    //A.block<3,PELV_N_SDFAST_U>(rowIdx,0) *= IK_FOOT_WEIGHT;
    //b.segment<3>(rowIdx) *= IK_FOOT_WEIGHT;
    A.block<3,PELV_N_SDFAST_U>(rowIdx,0) *= foot_q_weight;
    b.segment<3>(rowIdx) *= foot_q_weight;
    A.block<3,PELV_N_SDFAST_U>(rowIdx+3,0) *= foot_q_weight;
    b.segment<3>(rowIdx+3) *= foot_q_weight;
  
    // toe off
    if (cmd.footRef[side] == Foot::RefToe) {
      A.block<1,PELV_N_SDFAST_U>(rowIdx+4,0) *= 1e-3;
      b(rowIdx+4) *= 1e-3;
    }

    assert(!(b.segment<6>(rowIdx).array() == std::numeric_limits<double>::infinity()).any());
    rowIdx += 6;
  }

  ////////////////////////////////////////////////////////
  // root
  // A = I
  // b = pelvisd_d
  /*
  b.segment<3>(rowIdx) = Eigen::Map< const Eigen::Matrix<double,3,1> >(cmd.rootd);
  b.segment<3>(rowIdx+3) = Eigen::Map< const Eigen::Matrix<double,3,1> >(cmd.root_b_w); 
 
  Eigen::Quaterniond world_to_body = ikrs->rootq.inverse().normalized();
  Eigen::Quaterniond body_q_d = world_to_body * cmd.pelvQ;
  if (body_q_d.dot(Eigen::Quaterniond::Identity()) < 0) 
    neg_quat(body_q_d);
  
  Rotvec = quatMinus(Eigen::Quaterniond::Identity(), body_q_d);

  for (int i = 0; i < 6; i++) {
    // unspecified
    if (b(rowIdx+i) == std::numeric_limits<double>::infinity()) {
      A(rowIdx+i,i) = 2e-1;
      b(rowIdx+i) = 0; //2e-1*(-10*ikrs->rootd[i]);
    }
    else {
      A(rowIdx+i,i) = 1; // (5*comddWeight / rootddWeight);

      if (i >= 3) {
        b(rowIdx+i) -= IK_POS_RATE*Rotvec[i-3];
      }

      b(rowIdx+i) *= 1; // (5*comddWeight / rootddWeight);
    } 
  }
  A.block(rowIdx,0,6,6) *= IK_PELVIS_WEIGHT;
  b.segment<6>(rowIdx) *= IK_PELVIS_WEIGHT;

  assert(!(b.segment<6>(rowIdx).array() == std::numeric_limits<double>::infinity()).any());
  assert(!(b.segment<6>(rowIdx).array() == std::numeric_limits<double>::signaling_NaN()).any());
  
  rowIdx += 6;  
  */
  for (int i = 0; i < 3; i++) {
    A(rowIdx+i,i) = 2e-1;
    b(rowIdx+i) = 0;
  }
  A.block(rowIdx,0,3,3) *= IK_PELVIS_WEIGHT;
  b.segment<3>(rowIdx) *= IK_PELVIS_WEIGHT;
  rowIdx += 3;
 
  b.segment<3>(rowIdx) = Eigen::Map<const Eigen::Matrix<double,3,1> >(cmd.pelvW); 
  A.block<3,PELV_N_SDFAST_U>(rowIdx,0) = ikrs->Jpelvis.block<PELV_N_SDFAST_U,3>(0,3).transpose() * IK_PELVIS_WEIGHT;
  Rotvec = quatMinus(ikrs->rootq, cmd.pelvQ);

  for (int i = 0; i < 3; i++) {
    b(rowIdx+i) -= IK_POS_RATE*Rotvec[i];
  }

  b.segment<3>(rowIdx) *= IK_PELVIS_WEIGHT;
  rowIdx += 3;

  ////////////////////////////////////////////////////////
  // qd
  A.block<PELV_N_JOINTS,PELV_N_SDFAST_U>(rowIdx,0).setZero();
  b.segment<PELV_N_JOINTS>(rowIdx).setZero();
 
  b.segment<PELV_N_JOINTS>(rowIdx) = Eigen::Map<const Eigen::Matrix<double,PELV_N_JOINTS,1> > (cmd.jointsd);

  for (int i = 0; i < PELV_N_JOINTS; i++) {
    if (b(rowIdx+i) == std::numeric_limits<double>::infinity()) {
      A(rowIdx+i,6+i) = 2e-1;
      b(rowIdx+i) = ikrs->jointsd[i];
    }
    else {
      A(rowIdx+i,6+i) = 1;
      b(rowIdx+i) += IK_REF_GAIN*(cmd.joints[i] - ikrs->joints[i]);
    } 
  }
  A.block(rowIdx,6,PELV_N_JOINTS,PELV_N_JOINTS) *= IK_REF_WEIGHT;
  b.segment<PELV_N_JOINTS>(rowIdx) *= IK_REF_WEIGHT;

  assert(!(b.segment<PELV_N_JOINTS>(rowIdx).array() == std::numeric_limits<double>::infinity()).any());
  assert(!(b.segment<PELV_N_JOINTS>(rowIdx).array() == std::numeric_limits<double>::signaling_NaN()).any());
  
  rowIdx += PELV_N_JOINTS;

  ////////////////////////////////////////////////////////
  
  
  ////////////////////////////////////////////////////////
  // u torso
  // A = Jt
  // b = w
  b.segment<3>(rowIdx) = 
      Eigen::Map<const Eigen::Matrix<double,3,1> >(cmd.torsoW);
  if (!((b.segment<3>(rowIdx).array() == 
       std::numeric_limits<double>::infinity()).any()) && IK_TORSO_WEIGHT != 0) 
  {
    A.block<3,PELV_N_SDFAST_U>(rowIdx,0) = ikrs->Jtorso.block<PELV_N_SDFAST_U,3>(0,3).transpose() * IK_TORSO_WEIGHT;

    Rotvec = quatMinus(ikrs->utorsoq, cmd.torsoQ);

    for (int i = 0; i < 3; i++) {
      b(rowIdx+i) -= IK_COM_RATE*Rotvec[i];
    }

    b.segment<3>(rowIdx) *= IK_TORSO_WEIGHT;

    assert(!(b.segment<3>(rowIdx).array() == std::numeric_limits<double>::infinity()).any());
  
    rowIdx += 3; 
  }
  else
    b.segment<3>(rowIdx).setZero(); 

  /////////////////////////////////////////////////////////
  const int N_VARS = PELV_N_SDFAST_U;
  const int N_EQ_CON = 1;
  const int N_INEQ_CON = 2*PELV_N_JOINTS;
  Eigen::Matrix<double,N_VARS,1> x;

  Eigen::Matrix<double,N_VARS,N_VARS> G = 
    A.topRows(rowIdx).transpose()*A.topRows(rowIdx);
  Eigen::Matrix<double,N_VARS,1> g0 = 
    -A.topRows(rowIdx).transpose()*b.topRows(rowIdx);
  Eigen::Matrix<double,N_VARS,N_EQ_CON> CE;
  Eigen::Matrix<double,N_EQ_CON,1> ce0;
  CE.setZero();
  ce0.setZero();
  
  Eigen::Matrix<double,N_INEQ_CON,N_VARS> CI;
  Eigen::Matrix<double,N_INEQ_CON,1> ci0;
  CI.setZero();
  ci0.setZero();
  
  // joint limit:
  int idx;
  double l_lim;
  for (int i = 0; i < PELV_N_JOINTS; i++) {
    // never bent knee all the way straight
    l_lim = RobotState::jointsLimit[0][i];
    if (i == A_L_LEG_KNY)
      l_lim += 0.1;
    if (i == A_R_LEG_KNY)
      l_lim += 0.1;
    
    // for toe off
    if (i == A_L_LEG_KNY && ikrs->joints[i] < 0.4 && cmd.footRef[LEFT] == Foot::RefToe)
      l_lim = 0.4;
    else if (i == A_R_LEG_KNY && ikrs->joints[i] < 0.4 && cmd.footRef[RIGHT] == Foot::RefToe)
      l_lim = 0.4;

    //  qd*dt - low + q >= 0
    CI(2*i, 6+i) = ikrs->timeStep;
    ci0(2*i) = -l_lim + ikrs->joints[i];

    // -qd*dt - q + up >= 0
    CI(2*i+1, 6+i) = -ikrs->timeStep;
    ci0(2*i+1) = RobotState::jointsLimit[1][i] - ikrs->joints[i];
  }

  /*
  min 0.5 * x G x + g0 x
  s.t.
    CE^T x + ce0 = 0
    CI^T x + ci0 >= 0 
  */
  Eigen::solve_quadprog<N_VARS,N_EQ_CON,N_INEQ_CON>(G, g0, 
    CE, ce0, CI.transpose(), ci0, x);

  // integrate one step
  double qdot[PELV_N_SDFAST_Q] = {0};
  double q[PELV_N_SDFAST_Q] = {0};
  dvec_copy(q, ikrs->getSDFState(), PELV_N_SDFAST_Q);
  ikrs->model->sdu2qdot(x.data(), qdot);

  for (int i = 0; i < PELV_N_SDFAST_Q; i++) {
    q[i] += ikrs->timeStep * qdot[i];
  }

  // update rs
  ikrs->time += ikrs->timeStep;
  dvec_copy(ikrs->root, q, 3);
  dvec_copy(ikrs->rootd, qdot, 3);
  ikrs->rootq = Eigen::Quaterniond(q[PELV_S_Q3], q[PELV_S_Q0], q[PELV_S_Q1], q[PELV_S_Q2]);
  ikrs->rootq.normalize();
  dvec_copy(ikrs->root_b_w, qdot+3, 3);
  dvec_copy(ikrs->joints, q+6, PELV_N_JOINTS);
  dvec_copy(ikrs->jointsd, qdot+6, PELV_N_JOINTS);
  ikrs->computeSDFvars();
 

  /////////////////////////////////////////////////////////////
  // process output
  dvec_copy(output.root_d, ikrs->root, 3);
  output.rootq_d = ikrs->rootq;
  output.torsoq_d = ikrs->utorsoq;
  dvec_copy(output.rootd_d, ikrs->rootd, 3);
  dvec_copy(output.com_d, ikrs->com, 3);
  dvec_copy(output.comd_d, ikrs->comd, 3);
  dvec_copy(output.pelvW_d, ikrs->pelvisw, 3);
  dvec_copy(output.torsoW_d, ikrs->utorsow, 3);
  for (int side = 0; side < LR; side++) {
    dvec_copy(output.foot_d[side], ikrs->feet[side].w_sensor_pos, 6);
    dvec_copy(output.footd_d[side], ikrs->feet[side].w_sensor_vel, 6);
    output.footq_d[side] = ikrs->feet[side].w_q;
  }

  dvec_copy(output.joints_d, cmd.joints, N_JOINTS);
  dvec_copy(output.jointsd_d, cmd.jointsd, N_JOINTS);
  dvec_copy(output.joints_d, q+6, N_JOINTS);
  dvec_copy(output.jointsd_d, qdot+6, N_JOINTS);

  return true;
}

void SFPelvIkCon::addToLog(BatchLogger &logger) {

}
