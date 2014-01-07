/*
 * =====================================================================================
 *
 *       Filename:  Task2WalkingCon.h
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


#ifndef __TASK2_WALKING_CON
#define __TASK2_WALKING_CON

#include "spline.h"
#include "traj.h"
#include "RobotState.h"
#include "FootStep.h"
#include "Command.h"
#include <vector>

#include <string>
#include <fstream>
#include "Pose.h" 
#include "WalkingCon.h"

class Task2WalkingCon : public WalkingCon 
{ 
  public:
    enum Task2ScriptMode {
      Manual = 0,
      Ramp,
      ZigZag,
      FlatPyramid,
      TiltetPyramid,
      StepInPlace,
      BigStride,
      NUM_SCRIPT_MODE
    };

  protected:
    class ScriptData 
    {
      public:
        WalkingConTerrainMode mode;
        double dx;
        ScriptData(WalkingConTerrainMode m, double x) 
        {
          this->mode = m;
          this->dx = x;
        }
    };

    enum Task2WalkingConState {
      ShiftComToLeft = 0,
      ShiftComToRight,      // 1
      FixCopLeft,           // 2
      FixCopRight,          // 3
      LiftoffLeft,          // 4
      SwingUpLeft,          // 5
      SwingDownLeft,        // 6
      LiftoffRight,         // 7
      SwingUpRight,         // 8
      SwingDownRight,       // 9
      HoldLeftFoot,         // 10
      HoldRightFoot,        // 11
      Idle,                 // 12
      NUM_CON_STATE
    };

    /////////////////////////////////////////
    // flags 
    HatRobotState::UpperBodyMode _ubodyMode;

    bool _inited;
    int _state;
    int _contactState;

    // foot related flags
    bool _doToeOff;
    bool _swingLegJointMode;
    int _swingFoot;
    int _stanceFoot;
    int _swingDir;
    int _toeOffFoot;  
    WalkingCon::FootStage _footStage[LR];
   
    double _cop_comp_z;

    // terrain flags / ctr
    WalkingConTerrainMode _prevTerrainMode;
    WalkingConTerrainMode _terrainMode;
    // user can only set this, which won't be updated to current
    // terrain mode until the beginning of swing up. 
    WalkingConTerrainMode _nxtTerrainMode;
    double _lastZ;
    int _downCtr;
    int _upCtr;
    int _downSlopeCtr;
    int _stepCount;

    // double support flags
    int _shiftCOMstart;
    int _shiftCOMend;
    
    // com intergaors
    bool _com_int_flag[LR];

    // tele op take a step
    bool _takeStep; 
    
    /////////////////////////////////////////
    // tele op
    int _go;
    Pose _curStanceFootPose;
    double _curHeading;
    double _nxtOffset[6];
    double _nxtDx;
    
    // scripts for mode
    Task2ScriptMode _scriptType;
    std::list <ScriptData> _script;
    //std::list <WalkingConTerrainMode> _modeScript;

    // timing
    double _stateTime;
    double _ds_duration;
    double _ss_duration;
    double _liftoffDuration;
    
    Eigen::Vector3d _lastFeet[LR];
    double _com_d_over_foot[2];
    double _com_d_i_ini[LR][2];
    double _com_d_i[LR][2];
    double _foot_d_i[LR][2];
    double _realFoot_d[2][3];

    /////////////////////////////////////////
    // knots for interpolation
    knot_t _com0[3], _com1[3];
    knot_t _wl0, _wl1;
    knot_t _swgFoot0[3], _swgFoot1[3];
    Eigen::Quaterniond _pelvQ0, _pelvQ1;
    Eigen::Quaterniond _torsoQ0, _torsoQ1;
    Eigen::Quaterniond _swgQ0, _swgQ1;
    
    SFootStep _nxtFootStep;   // where the swing foot should land
    /////////////////////////////////////////
    // rs com offset knots
    knot_t _rs_com0[2], _rs_com1[2];

    void updateKnot(const RobotState &rs);
    void interp(const RobotState &rs, TrajPoint <12,0> &com_d);
    void interpFoot(const RobotState &rs, TrajPoint <7,0> &foot_d);
    
    Eigen::Vector3d computeCom_d(const RobotState &rs, int side, double dz);
    bool setupComDs(const RobotState &rs, int start, int end);
    // must call setupSwingLiftoff first
    bool setupComSs(const RobotState &rs);
   
    // process foot step
    void setupSwingLiftoff(const RobotState &rs);
    void setupSwingUp(const RobotState &rs);
    void setupSwingDown(const RobotState &rs);

    void fillDesireds(const RobotState &rs, const TrajPoint<12,0> &root, const TrajPoint<7,0> foot[LR]);
    
    void gotoState(const RobotState &rs, Task2WalkingConState nxt);
    //Eigen::Vector3d makeNxtRelFootStep(const RobotState &rs, int side);

    const static std::string conModeNames[NUM_CON_STATE];
    Eigen::Vector2d w_cop2b_cop(const RobotState &rs, int side);
    void updateNxtStep();

    void resetNxtOffset(int nxtSwing);
    void resetAfterStep(const RobotState &rs);

  public:
    const static std::string scriptModeNames[NUM_SCRIPT_MODE];

    ///////////////////////////////////////////////////////
    // params
    double com_z;
    double liftoff_rate;
    double liftoff_height;
    double swing_rate;
    double ds_flat_duration;
    double ds_block_duration;
    double ss_flat_duration;
    double ss_block_duration;
    double ss_lower_z_vel;
    double ss_height;

    // pd gains for computing desired acceleartion = p * (pos_d-pos) + d * (vec_d-vel)
    double fd_joint_p;  // joint positio
    double fd_joint_d;
    double fd_com_p[6];
    double fd_com_d[6];
    double fd_foot_p[6];
    double fd_foot_d[6]; 
    
    double com_int_i[2];
    double foot_int_i[2];
    double touch_down_fz_thres;
    ///////////////////////////////////////////////////////

    Task2WalkingCon();
    ~Task2WalkingCon();
    
    void init(const RobotState &rs);
    void buildLookup();
    int control(RobotState &rs, Command &cmd);

    inline bool hasInited() const { return _inited; }
    // getters
    inline int getPlannedContactState(double nimei) const { return _contactState; }
    inline int getConMode() const { return _state; }
    inline int getStepCount() const { return _stepCount; }
    inline bool isIdle() { return _state == Idle; }
    WalkingCon::FootStage getPlannedFootState(const RobotState &rs, int side) const { return _footStage[side]; }

    void addToLog(BatchLogger &logger);
    void setUpperbodyMode(HatRobotState::UpperBodyMode m);

    ////////////////////////////////////////////
    // teleop part
    SFootStep getNextStep() const;
    void setTerrainMode(WalkingConTerrainMode mode);
    void nudgeNxtStep(const double delta[6]);
    void takeStep();
    void nextSide(int side) {;}
    void toggleWalking(int side = RIGHT);

    void makeScript(Task2ScriptMode part);

    Task2ScriptMode getScriptMode() const { return _scriptType; }
    int getNumOfStepsLeft() const { return _script.size(); }
    WalkingConTerrainMode getTerrainMode() const { return _terrainMode; }
    bool isHalt() const;
    ////////////////////////////////////////////
    // not implemented
    inline const double *getPlannedCurRoot(const RobotState &rs) const { return NULL; }
    inline const double *getPlannedCurLeftFoot(const RobotState &rs) const { return NULL; }
    inline const double *getPlannedCurRightFoot(const RobotState &rs) const { return NULL; }
};

#endif
