/*
 * =====================================================================================
 *
 *       Filename:  Foot.h
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  03/13/2013 09:23:43 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  YOUR NAME (), 
 *        Company:  
 *
 * =====================================================================================
 */

#ifndef __FOOT_H
#define __FOOT_H

#include <eigen3/Eigen/Geometry>
#include <assert.h>

class Foot {
  public:
    enum FootRefType {
      RefCenter = 0,
      RefToe,
      RefHeel,
      RefSensor
    };
   
    /*
  private:
    FootRefType _refPoint;
    */

  public:
    // 0 -> left toe
    // 1 -> left heel
    // 2 -> right toe
    // 3 -> right heel
    static const double ankle_to_tip_offset[4][3];
    static const double ankle_to_sensor_offset[3];
    static const double ankle_to_center_offset[3];
    static const double ankle_to_toe_offset[3];
    static const double ankle_to_heel_offset[3];

    double vertices[4][6];    // for drawing

    Eigen::Quaterniond w_q;

    // in robot state, jacobians and stuff are computed
    // based on ankle_to_ref offset.
    // in ID, force is applied at ref point.
    // mid pos and velocity is only for the convinience
    // of high level controller
    double w_mid_pos[6];
    double w_mid_vel[6];
    double w_toe_pos[6];
    double w_toe_vel[6];
    double w_heel_pos[6];
    double w_heel_vel[6];
    
    // at the bottom of the foot, right below the ankle
    // where the ft sensor is.
    double w_sensor_pos[6];                    // at ref_point, last 3 are euler angles
    double w_sensor_vel[6];                    // at ref_point

    double w_F[6];
    double w_cop[2];
   
    double b_F[6];
    double b_cop[2];
    double b_w[3];

    Foot();
    Foot &operator= (const Foot &other);

    /*
    inline void setRefPoint(FootRefType ref) 
    {
      if (ref == RefToe) {
        _refPoint = RefToe;
        ankle_to_ref = ankle_to_toe_offset;
      }
      else if (ref == RefHeel) {
        _refPoint = RefHeel;
        ankle_to_ref = ankle_to_heel_offset;
      }
      else if (ref == RefCenter) {
        _refPoint = RefCenter;
        ankle_to_ref = ankle_to_center_offset;
      }
      else if (ref == RefAtlas) {
        _refPoint = RefAtlas;
        ankle_to_ref = ankle_to_atlas_offset;
      }
      else 
        assert(0);
    }

    inline FootRefType getRefPoint() const { return _refPoint; }
    */

    static void getBoundingBox(FootRefType ref, double box[4]);
};

#endif
