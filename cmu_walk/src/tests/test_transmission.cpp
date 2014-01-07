/*
 * =====================================================================================
 *
 *       Filename:  test_transmission.cpp
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  10/24/2013 08:23:28 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  YOUR NAME (), 
 *        Company:  
 *
 * =====================================================================================
 */


#include "Transmission.h"
#include "RobotState.h"
#include <stdio.h>

int main()
{
  double limit[2];
  //int joint = A_R_LEG_LHY;
  int joint = A_L_ARM_USY;
  double ang = -0.5;
  
  for (int i = 0; i < N_JOINTS; i++) {
    Transmission::getTorqueLimit(i, ang, limit, limit+1);
    printf("%s, @%g, l %g u %g\n", RobotState::joint_names[i].c_str(), ang, limit[0], limit[1]);
  }

  return 0;  
}
