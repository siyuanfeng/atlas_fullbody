/*
 * =====================================================================================
 *
 *       Filename:  BalancingCon.cpp
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  06/16/2013 12:43:09 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  YOUR NAME (), 
 *        Company:  
 *
 * =====================================================================================
 */

#include "Eigen_utils.hpp"
#include "BalancingCon.h"
#include <map>
#include <boost/thread.hpp>
#include "Utils.hpp"

BalancingCon::BalancingCon()
{
  buildLookup();

  _inited = false;
  _done = false;
 
  fd_joint_p = 150;
  fd_joint_d = 10;

  //ik_joint_p = 1;
  dvec_set(fd_com_p, 50, 6);
  dvec_set(fd_com_d, 10, 6);
  dvec_set(fd_foot_d, 0, 6);
  dvec_set(fd_foot_d, 0, 6);
  
  //dvec_set(ik_com_p, 0, 6);
  //dvec_set(ik_foot_p, 0, 6);

  stance_lax_d = stance_uay_d = stance_uhz_d = 1; 
}

BalancingCon::~BalancingCon()
{
  
}

void BalancingCon::init(const RobotState &rs)
{
  Pose pose;
  pose.pos = Eigen::Map<const Eigen::Vector3d> (rs.com);
  pose.rot = rs.rootq;
  ikCon->setToRobotState(rs);
  
  // set desireds to current
  //setDesiredUtorsoQ(zyx2quat(Eigen::Vector3d(0.11, 0, rs.root[5]))); 
  //setDesiredUtorsoQ(rs.rootq); 
  setDesiredCOM(pose, Twist(), Twist());
  setDesiredJoints(rs.joints, NULL, NULL);

  _inited = true;
  _neck_ay_d = 0.;
  
  printParams();
}

void BalancingCon::setDesiredJoints(const double joints_d[N_JOINTS], const double jointsd_d[N_JOINTS], const double jointsdd_d[N_JOINTS])
{
  for (int i = 0; i < N_JOINTS; i++) {
    if (joints_d) {
      _joints_d[i] = std::min(joints_d[i], RobotState::jointsLimit[1][i]);
      _joints_d[i] = std::max(joints_d[i], RobotState::jointsLimit[0][i]);
    }
    else 
      _joints_d[i] = 0;
    
    if (jointsd_d)
      _jointsd_d[i] = jointsd_d[i];
    else
      _jointsd_d[i] = 0;
    
    if (jointsdd_d)
      _jointsdd_d[i] = jointsdd_d[i];
    else
      _jointsdd_d[i] = 0;
  }
}

void BalancingCon::setDesiredCOM(const Pose &pose_d, const Twist &vel_d, const Twist &acc_d)
{
  // should probably check support polygon.
  _com_d = pose_d;
  _comd_d = vel_d;
  _comdd_d = acc_d;
}

void BalancingCon::fillDesireds(const RobotState &rs)
{
  // set up desired joint pos, vel and acc
  for (int i = 0; i < N_JOINTS; i++) {
    if (i >= A_L_LEG_UHZ && i <= A_R_LEG_LAX) { 
      ikcmd.joints[i] = INFINITY;
      ikcmd.jointsd[i] = INFINITY;
      idcmd.jointsdd[i] = INFINITY;
    }
    else if ((i >= A_BACK_LBZ && i <= A_BACK_UBX) && (rs.getType() != RobotState::TYPE_HAT_NS)) {
      ikcmd.joints[i] = INFINITY;
      ikcmd.jointsd[i] = INFINITY;
      idcmd.jointsdd[i] = INFINITY;
    }
    else {
      if (i == A_NECK_AY)
        ikcmd.joints[i] = _neck_ay_d;
      else 
        ikcmd.joints[i] = _joints_d[i];

      // FOR IK
      ikcmd.jointsd[i] = _jointsd_d[i];
      //cmd.jointsd_d[i] = _jointsd_d[i] + ik_joint_p*(cmd.joints_d[i] - rs.joints[i]);
      idcmd.jointsdd[i] = _jointsdd_d[i]
        + fd_joint_p*(ikcmd.joints[i]-rs.joints[i]) 
        + fd_joint_d*(ikcmd.jointsd[i]-rs.jointsd[i]); 
    }
  }
  
  //idcmd.wl = 0.5;
  double dist[2];
  for (int side = 0; side < LR; side++) {
    for (int i = 0; i < 2; i++)
      //dist[side] = sqrt((ikcmd.com[i]-ikcmd.foot[side][i])*(ikcmd.com[i]-ikcmd.foot[side][i]));
      dist[side] = sqrt((rs.com[i]-rs.feet[side].w_sensor_pos[i])*(rs.com[i]-rs.feet[side].w_sensor_pos[i]));
  }
  idcmd.wl = 1 - dist[LEFT] / (dist[LEFT]+dist[RIGHT]);
  idcmd.wl = (idcmd.wl > 0.9) ? 0.9 : idcmd.wl; 
  idcmd.wl = (idcmd.wl < 0.1) ? 0.1 : idcmd.wl; 

  //////////////////////////////////////////////////////////////////////////////////////////
  // root 
  for (int i = 0; i < 3; i++) {
    ikcmd.com[i] = _com_d.pos[i];
    // FOR IK
    ikcmd.comd[i] = _comd_d.linear[i];
    //cmd.comd_d[i] = _comd_d.linear[i] + ik_com_p[i]*(cmd.com_d[i]-rs.com[i]);
    idcmd.comdd[i] = _comdd_d.linear[i]
      + fd_com_p[i]*(ikcmd.com[i]-rs.com[i]) 
      + fd_com_d[i]*(ikcmd.comd[i]-rs.comd[i]);

    ikcmd.root[i] = INFINITY; 
    ikcmd.rootd[i] = INFINITY;
    idcmd.rootdd[i] = INFINITY;
  }

  // utorso orientation tracking
  ikcmd.torsoQ = _utorsoq_d;

  if (ikcmd.torsoQ.dot(rs.utorsoq) < 0)
    neg_quat(ikcmd.torsoQ);

  Eigen::Vector3d Rotvec = quatMinus(rs.utorsoq, ikcmd.torsoQ);

  for (int i = 0; i < 3; i++) {
    // FOR IK
    ikcmd.torsoW[i] = _comd_d.angular[i];
    //cmd.utorsow_d[i] = _comd_d.angular[i] - ik_com_p[i+3]*Rotvec[i];
    idcmd.torsoWd[i] = _comdd_d.angular[i] 
      - fd_com_p[i+3]*(Rotvec[i]) 
      + fd_com_d[i+3]*(ikcmd.torsoW[i]-rs.utorsow[i]);
  }

  // pelv orientation tracking
  ikcmd.pelvQ = _com_d.rot;
  if (ikcmd.pelvQ.dot(rs.rootq) < 0)
    neg_quat(ikcmd.pelvQ);
  
  Rotvec = quatMinus(rs.rootq, ikcmd.pelvQ);

  for (int i = 0; i < 3; i++) {
    ikcmd.pelvW[i] = _comd_d.angular[i];
    idcmd.pelvWd[i] = _comdd_d.angular[i]
      - fd_com_p[i+3]*(Rotvec[i]) 
      + fd_com_d[i+3]*(ikcmd.pelvW[i]-rs.rootd[i+3]);
  }

  /*
  Eigen::Quaterniond world_to_body = rs.rootq.inverse().normalized();
  Eigen::Quaterniond body_q_d = world_to_body * ikcmd.pelvQ;
  if (body_q_d.dot(Eigen::Quaterniond::Identity()) < 0) 
    neg_quat(body_q_d);
  Rotvec = quatMinus(Eigen::Quaterniond::Identity(), body_q_d);

  Eigen::Vector3d body_w_d = world_to_body.toRotationMatrix() * _comd_d.angular;
  Eigen::Vector3d body_wd_d = world_to_body.toRotationMatrix() * _comdd_d.angular;

  for (int i = 0; i < 3; i++) {
    // FOR IK
    ikcmd.root_b_w[i] = body_w_d[i];
    //cmd.root_b_w_d[i] = body_w_d[i] - ik_com_p[i+3]*Rotvec[i];
    idcmd.root_b_wd[i] = body_wd_d[i]
      - fd_com_p[i+3]*(Rotvec[i])
      + fd_com_d[i+3]*(ikcmd.root_b_w[i]-rs.root_b_w[i]);

    // ikcmd.rootd_d[i] = 0;
  }
  // world frame angular velocity and acceleration
  Eigen::Map <Eigen::Vector3d> root_w_wd_d(idcmd.pelvWd);
  Eigen::Map <Eigen::Vector3d> root_b_wd_d(idcmd.root_b_wd);
  root_w_wd_d = rs.rootq.toRotationMatrix() * root_b_wd_d;
  */

  // foot stuff
  Eigen::Vector3d zyx = quat2zyx(_com_d.rot);
  Eigen::Quaterniond q = zyx2quat(Eigen::Vector3d(0,0,zyx[ZZ]));
  for (int side = 0; side < LR; side++) 
  {
    ikcmd.footQ[side] = q;
    if (ikcmd.footQ[side].dot(rs.feet[side].w_q) < 0) 
      neg_quat(ikcmd.footQ[side]);
    Rotvec = quatMinus(rs.feet[side].w_q, ikcmd.footQ[side]);
    
    for (int i = 0; i < 6; i++) {
      ikcmd.foot[side][i] = rs.feet[side].w_sensor_pos[i];
      ikcmd.footd[side][i] = 0;
      idcmd.footdd[side][i] = 0;
      /*
      if (i < 3) {
        cmd.footdd_d[side][i] = 0;
      }
      else {
        cmd.footdd_d[side][i] = fd_foot_d[i]*(cmd.footd_d[side][i]-rs.feet[side].w_sensor_vel[i])
          -fd_foot_p[i]*(Rotvec[i-3]);
      }
      */
    }
  }

  double z = (rs.feet[LEFT].w_sensor_pos[ZZ] + rs.feet[RIGHT].w_sensor_pos[ZZ]) / 2.;
  double yaw_avg = 0;
  zyx = quat2zyx(rs.feet[LEFT].w_q);
  yaw_avg = zyx[2];
  zyx = quat2zyx(rs.feet[RIGHT].w_q);
  yaw_avg += zyx[2];
  yaw_avg /= 2.;
  Eigen::Matrix2d rot;
  rot << cos(yaw_avg), -sin(yaw_avg), sin(yaw_avg), cos(yaw_avg);

  // fix cop stuff
  for (int i = 0; i < 2; i++)  
    idcmd.cop[i] = rs.com[i] - idcmd.comdd[i] * (rs.com[ZZ] - z) / GRAVITY;
 
  // doesn't actually in world frame, is the offset, but not orientation
  splitCoP(idcmd.cop, idcmd.wl, rs.feet[LEFT].w_sensor_pos, rs.feet[RIGHT].w_sensor_pos, idcmd.w_cop[LEFT], idcmd.w_cop[RIGHT]);

  for (int side = 0; side < LR; side++) {
    Eigen::Vector2d tmp(idcmd.w_cop[side]);
    yaw_avg = getYaw(rs.feet[side].w_q);
    Eigen::Matrix2d world_to_foot;
    world_to_foot(0,0) = cos(-yaw_avg);
    world_to_foot(0,1) = -sin(-yaw_avg);
    world_to_foot(1,0) = sin(-yaw_avg);
    world_to_foot(1,1) = cos(-yaw_avg);
    Eigen::Map <Eigen::Vector2d> b_cop(idcmd.b_cop[side]);
    b_cop = world_to_foot * tmp;
    
    // add back the pos from foot
    idcmd.w_cop[side][XX] += rs.feet[side].w_sensor_pos[XX];
    idcmd.w_cop[side][YY] += rs.feet[side].w_sensor_pos[YY];
  }
}


int BalancingCon::control(RobotState &rs, Command &cmd)
{
  fillDesireds(rs);
  
  //dvec_copy(cmd.joints_d, ikcmd.joints, N_JOINTS);
  
  if (!idCon->ID(rs, idcmd, cmd))
    return -1;
  
  if (!ikCon->IK(ikcmd, cmd))
    return -1; 
  
  return 0;
}

void BalancingCon::buildLookup()
{
  _conName = "BAL";

  _lookup["fd_joint_p"] = &fd_joint_p;
  _lookup["fd_joint_d"] = &fd_joint_d;

  _lookup["stance_lax_d"] = &stance_lax_d;
  _lookup["stance_uay_d"] = &stance_uay_d;
  _lookup["stance_uhz_d"] = &stance_uhz_d;
  
  std::string name;

  for (int i = 0; i < 6; i++) {
    name = std::string("fd_com_p_");
    name.append(boost::to_string(i));
    _lookup[name] = fd_com_p+i;
    name = std::string("fd_com_d_");
    name.append(boost::to_string(i));
    _lookup[name] = fd_com_d+i;
    name = std::string("fd_foot_p_");
    name.append(boost::to_string(i));
    _lookup[name] = fd_foot_p+i;
    name = std::string("fd_foot_d_");
    name.append(boost::to_string(i));
    _lookup[name] = fd_foot_d+i; 
  }
}
