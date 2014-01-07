/*
 * =====================================================================================
 *
 *       Filename:  Walking.h
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  03/26/2013 04:01:33 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  YOUR NAME (), 
 *        Company:  
 *
 * =====================================================================================
 */

#ifndef __LIPM_WALKING_CON_H
#define __LIPM_WALKING_CON_H

#include "traj.h"
#include "spline.h"
#include "RobotState.h"
#include "FootStep.h"
#include "spline.h"
#include "lipm_planner.hpp"
#include <vector>

#include <string>
#include <fstream>
#include <boost/thread.hpp>
#include "Pose.h"
#include "WalkingCon.h"

class LipmWalkingCon : public WalkingCon {
  private:

    double _mass;
    bool _inited;
    // plan trajectory for these steps
    size_t _Nsteps;
    // foot step plan
    std::vector <SFootStep> _refinedPlan;
    
    // controls toe off in DSl and DSr phase
    bool _toeOffFlag;
    
    // foot and com trajectory
    // root.type = planned contactState
    // foot.type = mode (up / down etc.)
    Traj<9,0> _flTraj;
    Traj<9,0> _frTraj;
    Traj<7,0> _rootTraj;
    
    // testing com acc = lipm acc
    LipmVarHeightPlanner _lipm; 

    // picks up results from lipm thread, and wakes it up
    bool rePlan(const RobotState &rs, bool firstTime);
    
    // dummy
    bool refineFootSteps(const RobotState &rs, const std::vector <SFootStep> &fsPlan);
    void dumpPlan(const std::vector <SFootStep> &plan) const;
    
    /////////////////////////////////////////////////
    size_t _numStepsLeft;

    // lipm threading stuff
    boost::thread _lipmThread;
    // lock guards all these
    boost::mutex _lipmLock;
    // signals lipm thread to sleep / wake up
    boost::condition_variable _lipmCVar;
    // indicates new trajectories ready
    bool _lipmTrajReady;
    // used to kill lipm thread
    bool _lipmStop;
    // return code for generateTrajThreading
    bool _lipmRetCode;
    
    // first time LipmWalkingCon is initialized
    double _initTime;
    // foot step used to plan new trajectory
    Pose _lastFootStep[LR];
    
    // used together with updateFootSteps
    // new foot steps first cached here in the main thread, 
    // then used later in the lipm thread
    std::vector <SFootStep> _newPlanThreading;

    // lipm thread dumps new plan / trajectories here, 
    // the main thread picks these up in rePlan
    std::vector <SFootStep> _refinedPlanThreading;
    Traj<9,0> _flTrajThreading;
    Traj<9,0> _frTrajThreading;
    Traj<7,0> _rootTrajThreading;
    
    LipmVarHeightPlanner _lipmThreading; 

    void genTrajWorker();
    bool generateTrajThreading(const std::vector <SFootStep> &localPlan, double iniTime, 
      const Pose &com0, const Eigen::Vector3d &comd0, const Pose p0[LR], 
      size_t N_steps, Traj<9,0> &localFlTraj, Traj<9,0> &localFrTraj, Traj<7,0> &localRootTraj,
      LipmVarHeightPlanner &localLipm);

    void set_footpos_knots(double t0, double t1, const double p0[3], const double p1[3], double z, double s, knot_t *k0, knot_t *k1, knot_t *k_mid);
    void lookup_footpos(double t, const knot_t *k0, const knot_t *k1, const knot_t *k_mid, double *pos, double *vel, double *acc);

    /////////////////////////////////////////////////

  public:
    
    //////////////////////////////////////////////////////
    // parameters
    double lipm_step_duration;
    double lipm_step_height;
    double lipm_step_ds_duration;
    double lipm_zmp_x_dsc;
    
    double lipm_zmp_x_down;
    double lipm_zmp_x_up;
    double lipm_zmp_x_normal;
    double lipm_z_d_down;
    double lipm_z_d_up;
    double lipm_z_d_normal;

    double lipm_flat_swing_scale;
    double lipm_down_swing_scale;
    double lipm_up_swing_scale;
    double lipm_down_thres;
    double lipm_up_thres;
    double lipm_up_torso_pitch;

    double fd_joint_p;
    double fd_joint_d;
    double fd_com_p[6];
    double fd_com_d[6];
    double fd_foot_p[6];
    double fd_foot_d[6];
    
    /*
    double ik_joint_p;
    double ik_com_p[6];
    double ik_foot_p[6];
    */

    double fd_toeoff_y;
    double no_toeoff_sqr_vel_thresh;
    
    Eigen::Matrix<double,6,6> lipm_Q;
    Eigen::Matrix<double,3,3> lipm_R;
    //////////////////////////////////////////////////////
    
    LipmWalkingCon();
    ~LipmWalkingCon();

    void buildLookup();

    // init stuff and spawns lipm thread
    void init(const RobotState &rs);
    // kills lipm thread
    void stop();
    
    // wipe steps after idx and add new steps
    bool updateFootSteps(const RobotState &rs, const std::vector <SFootStep> &fsPlan);
   
    // regenerate trajectories if necessary, fills desireds, and
    // call QP to generate torques
    int control(RobotState &rs, Command &cmd);
    
    // fills desireds stuff for QP
    void fillDesireds(const RobotState &rs, const TrajPoint<7,0> &root, const TrajPoint<9,0> &fl, const TrajPoint<9,0> &fr);

    // getters
    inline const std::vector <SFootStep> &getRefinedPlan() const { return _refinedPlan; }
    inline bool isIdle() { return (getNumStepsLeft() == 0); }
    inline size_t getNumStepsLeft()
    { 
      boost::mutex::scoped_lock lock(_lipmLock);
      return _numStepsLeft;
    }
    
    inline bool hasInited() const { return _inited; }
    
    WalkingCon::FootStage getPlannedFootState(const RobotState &rs, int side) const { return WalkingCon::Planted; }
    
    // used in updateRobotState in gazebo / ros to update
    // contact state, could be swapped out by Ben's filter stuff
    inline int getPlannedContactState(double time) const { return _rootTraj[_rootTraj.getIdx(time)].type; }
    inline int getPlannedMode(double time) const { return _flTraj[_flTraj.getIdx(time)].type; }
    inline const double *getPlannedCurRoot(const RobotState &rs) const { return _rootTraj[_rootTraj.getIdx(rs.time)].pos; }
    inline const double *getPlannedCurLeftFoot(const RobotState &rs) const { return _flTraj[_flTraj.getIdx(rs.time)].pos; }
    inline const double *getPlannedCurRightFoot(const RobotState &rs) const { return _frTraj[_frTraj.getIdx(rs.time)].pos; }
    void addToLog(BatchLogger &logger) {;}
};        

#endif
