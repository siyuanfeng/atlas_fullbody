/*
 * =====================================================================================
 *
 *       Filename:  FootStep.h
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  03/19/2013 10:56:50 AM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  sf 
 *        Company:  
 *
 * =====================================================================================
 */

#ifndef __FOOT_STEP_H
#define __FOOT_STEP_H

#include "drc_common_defines.h"
#include "Eigen_utils.hpp"
#include <eigen3/Eigen/Geometry>
#include <assert.h>
#include "Logger.h"
#include "Pose.h"

class SFootStep {
  public:
    // SSL, pos q are the next RIGHT foot hold
    // SSR, pos q are the next LEFT foot hold 
    // DSc, nothing moves, ignores pos q
    int type;
    double time;    
    Pose pose;
    
    SFootStep() 
    {
      type = -1;
      time = -1;
    }
    
    SFootStep(int s, double t) 
    {
      type = s;
      time = t;
       
      assert(s == DSc);
    }
    
    SFootStep(int s, double t, Pose &p) 
    {
      type = s;
      time = t;
      
      pose = p;
    }

    SFootStep(int s, double t, const double *pos) 
    {
      type = s;
      time = t;

      pose.pos = Eigen::Map<const Eigen::Vector3d> (pos);
      pose.rot = euler2quat(Eigen::Map<const Eigen::Vector3d> (pos+3));
    }

    /*
    SFootStep(int s, double t, const double p[6]) 
    {
      type = s;
      time = t;
      
      assert(p != NULL);
      for (int i = 0; i < 6; i++)
        pos[i] = p[i];
      
      euler2quat(&(pos[3]), quat);
    }
    */

    SFootStep(const SFootStep &other) 
    {
      type = other.type;
      time = other.time;
      
      pose = other.pose;
    }

    SFootStep &operator=(const SFootStep &other) {
      if (this == &other)
        return *this;
      type = other.type;
      time = other.time;
      pose = other.pose;
      return *this;
    }

    void addToLog(Logger &logger) {
      logger.add_datapoint("FS.pos[X]", "m", pose.pos.data()); 
      logger.add_datapoint("FS.pos[Y]", "m", pose.pos.data()+1); 
      logger.add_datapoint("FS.pos[Z]", "m", pose.pos.data()+2);
      logger.add_quat("FS.Q", &(pose.rot));
    }
};

#endif
