/*
 * =====================================================================================
 *
 *       Filename:  test_ik.cpp
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  10/16/2013 10:57:05 AM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  YOUR NAME (), 
 *        Company:  
 *
 * =====================================================================================
 */

#include <cmu_walk/Utils.hpp>
#include <cmu_walk/ik_controller.hpp>
#include <cmu_walk/drc_hat_defines.h>
#include <iostream>

void fill_cmd(const RobotState &rs, Command &cmd)
{
  for (int i = 0; i < N_JOINTS; i++) {
    cmd.joints_d[i] = Command::BDI_joints[i];
    cmd.jointsd_d[i] = 0;
    cmd.jointsdd_d[i] = 0;
  }
  for (int i = 0; i < 3; i++) {
    cmd.com_d[i] = rs.com[i];
  }
  cmd.com_d[ZZ] -= 0.5;

  cmd.utorsoq_d = rs.utorsoq;
  cmd.rootq_d = rs.rootq;
  for (int side = 0; side < LR; side++) {
    cmd.footq_d[side] = rs.feet[side].w_q;
    dvec_copy(cmd.foot_d[side], rs.feet[side].w_pos, 3);
  }
  
}

int main()
{
  //PelvRobotState rs;
  //PelvIkCon ik;
  HatRobotState rs;
  HatIkCon ik;
  for (int i = 0; i < 3; i++)
    ik.ik_com_p[i] = 0;
  ik.ik_joint_p = 50;
  ik.printParams();

  double joints[N_JOINTS] = {0};
  double jointsd[N_JOINTS] = {0};
  double pos[3] = {0};
  double rootd[3] = {0};
  double root_b_w[3] = {0};
  Eigen::Quaterniond q = Eigen::Quaterniond::Identity();

  ((RobotState &)rs).computeSDFvars(pos, q.coeffs().data(), rootd, root_b_w, joints, jointsd);
 
  Command cmd;

  for (int i = 0; i < 5000; i++) {
    fill_cmd(rs, cmd);
    assert(ik.IK(rs, cmd));
  }

  for (int i = 0; i < 35; i++) {
    if (i < 6)
      printf("%10s %5g\n", "q", cmd.ik_q[i]);
    else 
      printf("%10s %5g\n", RobotState::joint_names[i-6].c_str(), cmd.ik_q[i]);
  }

  return 0;
}
