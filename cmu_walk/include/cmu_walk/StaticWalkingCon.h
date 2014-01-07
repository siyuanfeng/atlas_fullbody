/*
 * =====================================================================================
 *
 *       Filename:  StaticWalkingCon.h
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  05/25/2013 10:53:55 AM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  YOUR NAME (), 
 *        Company:  
 *
 * =====================================================================================
 */


#ifndef __STATIC_WALKING_CON
#define __STATIC_WALKING_CON

#include "traj.h"
#include "RobotState.h"
#include "Command.h"
#include <vector>

#include <string>
#include <fstream>
#include "Pose.h" 
#include "WalkingCon.h"

class StaticWalkingCon : public WalkingCon {
  private:

    enum StaticWalkingConState {
      ShiftComToLeft = 0,
      ShiftComToRight,
      LiftoffLeft,
      LiftLeftFoot,
      LowerLeftFoot,
      TouchDownLeft,
      LiftoffRight,
      LiftRightFoot,
      LowerRightFoot,
      TouchDownRight,
      Idle,
      NUM_CON_STATE
    };


    // controls _state -> idle mode
    int _stopping;
    bool _idle;
    bool _inited;

    bool _mudMode;
    int _terrainMode;
    double _stateTime;
    int _state;
    int _contactState;
    int _stepCount;
    double _heading;
    double _distTraveled;
    //Eigen::Vector2d _iniPos;
    double _maxDist;
    
    Eigen::Vector3d _swingFeetPosIntegral[LR];
    Eigen::Vector3d _swingFeetRotIntegral[LR];

    Pose _stanceFeet[LR];
    Pose _curFeet[LR];
    Pose _nxtFootStep;
    // 3 pos, 4 quat, 1 weight distribution
    Traj <8,0> _dsComTraj;

    //void updateComTraj(const Eigen::Vector3d com, const Eigen::Vector3d &comd, const Eigen::Vector3d &foot, size_t idx);
    bool setupDs(const RobotState &rs, int start, int end);
    
    // Traj <7,0> _ssFootTraj;
    // bool setupLift(const RobotState &rs, int dir);
    // bool setupLower(const RobotState &rs, int dir);

    void fillDesireds(const RobotState &rs, const TrajPoint<8,0> &root, const TrajPoint<7,0> &foot);
    
    void gotoState(StaticWalkingConState nxt);
    Pose makeNxtStraightFootStep(const RobotState &rs, int side);

    //const static std::string terrainModeNames[NUM_TERRAIN_MODE];
    const static std::string conModeNames[NUM_CON_STATE];
  public:
    ///////////////////////////////////////////////////////
    // params
    double step_length;
    double step_width;
    double com_z;
    double ds_duration;
    double up_ds_duration;
    double ss_duration;
    double ss_lower_z_rate;
    double ss_height;

    // pd gains for computing desired acceleartion = p * (pos_d-pos) + d * (vec_d-vel)
    double fd_joint_p;  // joint positio
    double fd_joint_d;
    double fd_com_p[6];
    double fd_com_d[6];
    double fd_foot_p[6];
    double fd_foot_d[6]; 
    double fd_foot_i[6];

    double up_foot_pitch;
    double up_torso_pitch;
    double up_ss_height;
    double up_step_length;
    double up_com_z;

    // damping gain on the ankle joints and hip yaw 
    // trq -= d * jointd
    double stance_lax_d;
    double stance_uay_d;
    double stance_uhz_d;
    // percentage of m*g that triggers transition from swing down to double support
    double touch_down_fz_thres;
    // percentage of m*g that triggers transition from double support to swing up
    double lift_off_fz_thres;
    // upper and lower limit for w (Fz_left / F_total)
    double w_high_thres;
    double w_low_thres;
    ///////////////////////////////////////////////////////

    StaticWalkingCon();
    ~StaticWalkingCon();
    
    void init(const RobotState &rs);
    void switchToMudMode();
    void buildLookup();
    int control(RobotState &rs, Command &cmd);
    void startWalking(const RobotState &rs, double heading, double dist);
    void stop();

    inline bool hasInited() const { return _inited; }
    inline void setHeading(double yaw) { _heading = yaw; }
    inline double getHeading() const { return _heading; }
    // getters
    inline int getPlannedContactState(double nimei) const { return _contactState; }
    inline int getConMode() const { return _state; }
    inline bool isIdle() { return _state == Idle; }
    
    WalkingCon::FootStage getPlannedFootState(const RobotState &rs, int side) const { return Planted; }

    ////////////////////////////////////////////
    // not implemented
    inline const double *getPlannedCurRoot(const RobotState &rs) const { return NULL; }
    inline const double *getPlannedCurLeftFoot(const RobotState &rs) const { return NULL; }
    inline const double *getPlannedCurRightFoot(const RobotState &rs) const { return NULL; }
    void addToLog(BatchLogger &logger) {;}
};

#endif
