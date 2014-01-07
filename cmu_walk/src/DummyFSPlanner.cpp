/*
 * =====================================================================================
 *
 *       Filename:  DummyFSPlanner.cpp
 *
 *    Description:  
 *
 *
 *        Version:  1.0
 *        Created:  03/27/2013 08:34:26 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  sf 
 *        Company:  
 *
 * =====================================================================================
 */

#include <math.h>
#include "Eigen_utils.hpp"
#include "DummyFSPlanner.h"
#include "drc_common_defines.h"
//#include "quaternion.h"
#include <stdio.h>
#include <assert.h>

#include <iostream>
#include <fstream>

void fsFromFile(const char *name, std::vector <SFootStep> &foot_steps)
{
  std::ifstream in(name);
  Pose pose;
  int state;
  double tmp;
  
  while (in.good()) {
    // xy
    in >> pose.pos[0];
    in >> pose.pos[1];
    pose.pos[2] = 0.1;
    // yaw
    in >> tmp; 
    pose.rot = Eigen::AngleAxisd(tmp, Eigen::Vector3d::UnitZ());

    std::cout << pose.pos[0] << " " <<  pose.pos[1] << " " << tmp;

    // state
    in >> tmp;
    state = (int)tmp;

    std::cout << state << std::endl;
    if (state == 0)
      state = SSR;
    else if (state == 1)
      state = SSL;
    else 
      assert(0);
    
    // cost
    // in >> tmp;
    
    foot_steps.push_back(SFootStep(state, 0, pose));
  }

  in.close();
}

/*
void fsFromFile(const char *name, std::vector <SFootStep> &foot_steps)
{
  std::ifstream in(name);
  Pose pose;
  int state;
  double tmp;
  
  while (in.good()) {
    // xy
    in >> pose.pos[0];
    in >> pose.pos[1];
    // yaw
    in >> tmp; 
    pose.rot = Eigen::AngleAxisd(tmp, Eigen::Vector3d::UnitZ());

    // state
    in >> tmp;
    state = (int)tmp;

    std::cout << state << std::endl;
    if (state == 0)
      state = SSR;
    else if (state == 1)
      state = SSL;
    else 
      assert(0);
    
    // cost
    in >> tmp;
    
    foot_steps.push_back(SFootStep(state, 0, pose));
  }

  in.close();
}
*/

// start and end are pelvis location
void make_straight_fs_plan(const double start[6], const double end[6], double step_len, std::vector <SFootStep> &foot_steps, int firstState, double step_width)
{
  double center[2] = {start[XX], start[YY]};

  Eigen::Vector2d travelDir(end[0]-start[0], end[1]-start[1]);
  travelDir.normalize();
  Eigen::Vector2d headingDir(cos(end[5]), sin(end[5]));

  double xy_dist = sqrt((end[XX]-start[XX])*(end[XX]-start[XX]) + (end[YY]-start[YY])*(end[YY]-start[YY]));
  
  int N = ceil(xy_dist / step_len);
 
  fprintf(stderr, "heading %g, dot %g \n", end[5], travelDir.dot(headingDir));

  if (travelDir.dot(headingDir) < 0)
    step_len = -xy_dist / (double)N;
  else
    step_len = xy_dist / (double)N;

  // initial step
  /*
  int state;
  if (end[5] > start[5])
    state = SSR;
  else
    state = SSL;
  */
  int state = firstState;
  Pose pose;

  //double ang[3] = {0, -0.2, end[5]};

  //foot_steps.clear();
  printf("STRAIGHT PLAN %d steps\n", N);

  for (int i = 1; i <= N; i++) {
    
    /*
    if (i >= 3)
      step_len = 0.5;
    */

    center[XX] = start[XX] + i*step_len*cos(end[5]);
    center[YY] = start[YY] + i*step_len*sin(end[5]);
    
    if (state == SSL) {
      pose.pos[XX] = center[XX] + step_width*cos(end[5]-M_PI/2);
      pose.pos[YY] = center[YY] + step_width*sin(end[5]-M_PI/2);
      pose.pos[ZZ] = start[ZZ];
      pose.rot = Eigen::AngleAxisd(end[5], Eigen::Vector3d::UnitZ());
      //zyx2quat(ang, pose.rot);

      foot_steps.push_back(SFootStep(state, 0, pose));
      state = SSR;
    }
    else {
      pose.pos[XX] = center[XX] + step_width*cos(end[5]+M_PI/2);
      pose.pos[YY] = center[YY] + step_width*sin(end[5]+M_PI/2);
      pose.pos[ZZ] = start[ZZ];
      
      pose.rot = Eigen::AngleAxisd(end[5], Eigen::Vector3d::UnitZ());
      //zyx2quat(ang, pose.rot);
      
      foot_steps.push_back(SFootStep(state, 0, pose));
      state = SSL;
    }
  }

  // last step
  if (state == SSL) {
    pose.pos[XX] = center[XX] + step_width*cos(end[5]-M_PI/2);
    pose.pos[YY] = center[YY] + step_width*sin(end[5]-M_PI/2);
    pose.pos[ZZ] = start[ZZ];
    pose.rot = Eigen::AngleAxisd(end[5], Eigen::Vector3d::UnitZ());
    //zyx2quat(ang, pose.rot);

    foot_steps.push_back(SFootStep(state, 0, pose));
  }
  else {
    pose.pos[XX] = center[XX] + step_width*cos(end[5]+M_PI/2);
    pose.pos[YY] = center[YY] + step_width*sin(end[5]+M_PI/2);
    pose.pos[ZZ] = start[ZZ];
    pose.rot = Eigen::AngleAxisd(end[5], Eigen::Vector3d::UnitZ());
    //zyx2quat(ang, pose.rot);

    foot_steps.push_back(SFootStep(state, 0, pose));
  }
}

void make_turn_fs_plan(const double start[6], const double end[6], std::vector <SFootStep> &foot_steps, int firstState)
{
  double center[2];

  double yaw_dist = end[5]-start[5];
  double yaw_per_step = M_PI/8.;

  int N = ceil(fabs(yaw_dist) / (yaw_per_step));
  yaw_per_step = yaw_dist / (double)N;

  printf("turning %d steps\n", N);

  //double pos[6] = {0};
  Pose pose;
  
  int state;
  if (firstState == DSc) {
    // turn left
    if (start[5] < end[5]) {
      state = SSR;
    }
    else {
      state = SSL;
    }
  }
  else {
    state = firstState;
  }

  //foot_steps.clear();

  center[XX] = end[XX];
  center[YY] = end[YY];
  double yaw = start[5];

  for (int i = 1; i <= N; i++) {
    if (state == SSL) {
      yaw += yaw_per_step;

      pose.pos[XX] = center[XX] - 0.089*cos(yaw+M_PI/2.);
      pose.pos[YY] = center[YY] - 0.089*sin(yaw+M_PI/2.);
      pose.pos[ZZ] = start[ZZ];
      pose.rot = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());

      foot_steps.push_back(SFootStep(state, 0, pose));
      state = SSR;
    }
    else {
      yaw += yaw_per_step;

      pose.pos[XX] = center[XX] + 0.089*cos(yaw+M_PI/2.);
      pose.pos[YY] = center[YY] + 0.089*sin(yaw+M_PI/2.);
      pose.pos[ZZ] = start[ZZ];
      pose.rot = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());

      foot_steps.push_back(SFootStep(state, 0, pose)); 
      state = SSL;
    }
  }

  if (state == SSL) {
    pose.pos[XX] = center[XX] - 0.089*cos(yaw+M_PI/2.);
    pose.pos[YY] = center[YY] - 0.089*sin(yaw+M_PI/2.);
    pose.pos[ZZ] = start[ZZ];
    pose.rot = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());

    foot_steps.push_back(SFootStep(state, 0, pose));
  }
  else {
    pose.pos[XX] = center[XX] + 0.089*cos(yaw+M_PI/2.);
    pose.pos[YY] = center[YY] + 0.089*sin(yaw+M_PI/2.);
    pose.pos[ZZ] = start[ZZ];
    pose.rot = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());

    foot_steps.push_back(SFootStep(state, 0, pose)); 
  }
} 
