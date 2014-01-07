/*
 * =====================================================================================
 *
 *       Filename:  BalancingCon.h
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  06/16/2013 12:37:15 AM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  YOUR NAME (), 
 *        Company:  
 *
 * =====================================================================================
 */

#ifndef __BALANCING_CON_H
#define __BALANCING_CON_H

#include "id_controller.hpp"
#include "WalkingCon.h"
#include "Pose.h"


class BalancingCon : public WalkingCon {
  private:
    void fillDesireds(const RobotState &rs);
    
    bool _inited;
    bool _done;
    
    double _joints_d[N_JOINTS];
    double _jointsd_d[N_JOINTS];
    double _jointsdd_d[N_JOINTS];
    Eigen::Quaterniond _utorsoq_d;
    Pose _com_d;
    Twist _comd_d;
    Twist _comdd_d;

  public:
    ///////////////////////////////////////////////////////
    // params
    double fd_joint_p;  // joint positio
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
    
    double stance_lax_d;
    double stance_uay_d;
    double stance_uhz_d;
    
    ///////////////////////////////////////////////////////
    
    BalancingCon();
    ~BalancingCon();
   
    void setDesiredUtorsoQ(const Eigen::Quaterniond q) {_utorsoq_d = q;}
    void setDesiredCOM(const Pose &pose_d, const Twist &vel_d, const Twist &acc_d);
    void setDesiredJoints(const double joints_d[N_JOINTS], const double jointsd_d[N_JOINTS], const double jointsdd_d[N_JOINTS]);

    void buildLookup();
    void init(const RobotState &rs);
    int control(RobotState &rs, Command &cmd);
    bool hasInited() const { return _inited; }
    int getPlannedContactState(double time) const { return DSc; }
    bool isIdle() { return _done; }
    
    WalkingCon::FootStage getPlannedFootState(const RobotState &rs, int side) const { return WalkingCon::Planted; }
    
    // not implemented
    const double *getPlannedCurRoot(const RobotState &rs) const { return NULL; }
    const double *getPlannedCurLeftFoot(const RobotState &rs) const { return NULL; }
    const double *getPlannedCurRightFoot(const RobotState &rs) const { return NULL; }
    void addToLog(BatchLogger &logger) {;}
};

#endif
