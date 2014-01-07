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

int main()
{
  PelvRobotState pelvRs;
  //PelvIkCon ik;
  HatRobotState rs;
  
  double joints[N_JOINTS] = 
  {
    0, 0, 0,
    0,
    0, 0.06, -0.23, 0.58, -0.31, -0.06,
    0, -0.06, -0.23, 0.58, -0.31, 0.06,
    0.3, -1.3, 2, 0.5, 0, 0,
    0.3, 1.3, 2, -0.5, 0, 0
  };
  double jointsd[N_JOINTS] = {0};
  double pos[3] = {0};
  double rootd[3] = {0};
  double root_b_w[3] = {0};
  Eigen::Quaterniond q = Eigen::Quaterniond::Identity();

  ((RobotState &)rs).computeSDFvars(pos, q.coeffs().data(), rootd, root_b_w, joints, jointsd);
  
  ((RobotState &)pelvRs).computeSDFvars(pos, q.coeffs().data(), rootd, root_b_w, joints, jointsd);

  printf("hat %g com %g %g %g\n", rs.m, rs.com[XX], rs.com[YY], rs.com[ZZ]);
  printf("pelv %g com %g %g %g\n", pelvRs.m, pelvRs.com[XX], pelvRs.com[YY], pelvRs.com[ZZ]);

  printf("l foot, %g %g %g\n", pelvRs.tfRoot[XX], pelvRs.tfRoot[YY], pelvRs.tfRoot[ZZ]); 
  printf("l foot, %g %g %g\n", pelvRs.feet[LEFT].w_sensor_pos[XX], pelvRs.feet[LEFT].w_sensor_pos[YY], pelvRs.feet[LEFT].w_sensor_pos[ZZ]); 

  //Eigen::Map <Eigen::Vector3d> rt(pelvRs.tfRoot);
  Eigen::Map <Eigen::Vector3d> rt(pelvRs.tfRoot);
  Eigen::Map <Eigen::Vector3d> fg_m(pelvRs.feet[LEFT].w_mid_pos);
  Eigen::Map <Eigen::Vector3d> fg_s(pelvRs.feet[LEFT].w_sensor_pos);

  std::cout << fg_m - rt << std::endl;
  std::cout << fg_s - rt << std::endl;

  /*
  printf("root: %g %g %g\n", rs.root[XX], rs.root[YY], rs.root[ZZ]);
  
  double btj[3] = {0};
  rs.model->sdgetbtj(HAT_S_JOINT_L_LEG_UHZ, btj);
  rs.model->sdpos(HAT_S_BODY_L_UGLUT, btj, pos);
  
  printf("l uhz: %g %g %g\n", pos[XX], pos[YY], pos[ZZ]);

  rs.model->sdgetbtj(HAT_S_JOINT_L_LEG_MHX, btj);
  rs.model->sdpos(HAT_S_BODY_L_LGLUT, btj, pos);

  printf("l mhx: %g %g %g\n", pos[XX], pos[YY], pos[ZZ]);

  rs.model->sdgetbtj(HAT_S_JOINT_L_LEG_LHY, btj);
  rs.model->sdpos(HAT_S_BODY_L_ULEG, btj, pos);

  printf("l lhy: %g %g %g\n", pos[XX], pos[YY], pos[ZZ]);
  */

  return 0;
}
