/*
 * =====================================================================================
 *
 *       Filename:  Pose.h
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  05/11/2013 12:45:24 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  YOUR NAME (), 
 *        Company:  
 *
 * =====================================================================================
 */

#ifndef __POSE_H
#define __POSE_H

#include <eigen3/Eigen/Geometry>

class Pose {
  public:
    Eigen::Vector3d pos;
    Eigen::Quaterniond rot;
 
    Pose() {
      pos = Eigen::Vector3d::Zero();  
      rot = Eigen::Quaterniond::Identity();
    }

    Pose(const Eigen::Vector3d &p, const Eigen::Quaterniond &r) {
      pos = p;
      rot = r;
    }
    
    Pose(const Pose &pose) {
      pos = pose.pos;
      rot = pose.rot;
    }
    
    Pose &operator=(const Pose &other) {
      if (this == &other)
        return *this;
      pos = other.pos;
      rot = other.rot;
      return *this;
    }
};

class Twist {
  public:
    Eigen::Vector3d linear;
    Eigen::Vector3d angular;
    
    Twist() {
      linear = Eigen::Vector3d::Zero();
      angular = Eigen::Vector3d::Zero();
    }
    
    Twist(const Eigen::Vector3d &v, const Eigen::Vector3d &w) {
      this->linear = v;
      this->angular = w;
    }
    
    Twist(const Twist &other) {
      linear = other.linear;
      angular = other.angular;
    }
    
    Twist &operator=(const Twist &other) {
      if (this == &other)
        return *this;
      linear = other.linear;
      angular = other.angular;
      return *this;
    }  
};

#endif
