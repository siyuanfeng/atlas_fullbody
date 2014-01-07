/*
 * =====================================================================================
 *
 *       Filename:  Foot.cpp
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  03/13/2013 09:27:51 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  YOUR NAME (), 
 *        Company:  
 *
 * =====================================================================================
 */


#include "Foot.h"
#include "Utils.hpp"

const double Foot::ankle_to_sensor_offset[3] = {0, 0, -0.075};
const double Foot::ankle_to_center_offset[3] = {0.045, 0, -0.075};
const double Foot::ankle_to_toe_offset[3] = {0.175, 0, -0.075};
const double Foot::ankle_to_heel_offset[3] = {-0.085, 0, -0.075};
 
const double Foot::ankle_to_tip_offset[4][3] = {
  {0.175, 0.062, -0.075}, 
  {-0.085, 0.062, -0.075}, 
  {0.175, -0.062, -0.075}, 
  {-0.085, -0.062, -0.075}, 
}; 

Foot::Foot()
{
  for (int i = 0; i < 4; i++) {
    dvec_set(vertices[i], 0, 6);
  }

  w_q = Eigen::Quaterniond::Identity();

  dvec_set(w_mid_pos, 0, 6);
  dvec_set(w_mid_vel, 0, 6);
  
  dvec_set(w_toe_pos, 0, 6);
  dvec_set(w_toe_vel, 0, 6);
  
  dvec_set(w_heel_pos, 0, 6);
  dvec_set(w_heel_vel, 0, 6);

  dvec_set(w_sensor_pos, 0, 6);
  dvec_set(w_sensor_vel, 0, 6);

  dvec_set(w_F, 0, 6);
  dvec_set(b_F, 0, 6);
  
  dvec_set(w_cop, 0, 2);
  dvec_set(b_cop, 0, 2);
  dvec_set(b_w, 0, 3);

  //setRefPoint(RefToe);
  //setRefPoint(RefHeel);
  //setRefPoint(RefCenter);
}

Foot &Foot::operator= (const Foot &other)
{
  if (this == &other)
    return *this;
  
  for (int i = 0; i < 4; i++) {
    dvec_copy(vertices[i], other.vertices[i], 6);
  }
  
  w_q = other.w_q;

  dvec_copy(w_mid_pos, other.w_mid_pos, 3);
  dvec_copy(w_mid_vel, other.w_mid_vel, 3);

  dvec_copy(w_toe_pos, other.w_toe_pos, 3);
  dvec_copy(w_toe_vel, other.w_toe_vel, 3);
  
  dvec_copy(w_heel_pos, other.w_heel_pos, 3);
  dvec_copy(w_heel_vel, other.w_heel_vel, 3);
  
  dvec_copy(w_sensor_pos, other.w_sensor_pos, 6);
  dvec_copy(w_sensor_vel, other.w_sensor_vel, 6);

  dvec_copy(w_F, other.w_F, 6);
  dvec_copy(w_cop, other.w_cop, 2);

  dvec_copy(b_F, other.b_F, 6);
  dvec_copy(b_cop, other.b_cop, 2);
  dvec_copy(b_w, other.b_w, 3);

  //setRefPoint(other.getRefPoint());

  return *this;
}

void Foot::getBoundingBox(FootRefType ref, double box[4])
{
  switch (ref) {
    case RefCenter:
      box[0] = 0.13-0.015;
      box[1] = -0.13+0.015;
      box[2] = 0.062-0.01; 
      box[3] = -0.062+0.01; 
      break;
    case RefToe:
      //box[0] = 0-0.015;
      //box[1] = -0.26+0.015;

      // for toe off
      box[0] = 0;
      box[1] = 0;
      box[2] = 0.062-0.015; 
      box[3] = -0.062+0.015; 
      break;
    case RefHeel:
      //box[0] = 0.26-0.015;
      //box[1] = 0+0.015;

      // for heel strike
      box[0] = 0;
      box[1] = 0;
      box[2] = 0.062-0.015; 
      box[3] = -0.062+0.015; 
      break;
    case RefSensor:
      // measured from force plate and foot sensor
      box[0] = 0.175-0.03;      // 0.1457
      box[1] = -0.085+0.025;    // -0.063 on left, -0.066 on right
      box[2] = 0.062-0.02;      // guessed 
      box[3] = -0.062+0.02;
      break;
    default:
      box[0] = 0;
      box[1] = 0;
      box[2] = 0;
      box[3] = 0;
      break;
  }
}
