#ifndef __ATLAS_STATIC_WALKING_CON
#define __ATLAS_STATIC_WALKING_CON

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

class AtlasStaticWalkingCon : public WalkingCon {
  protected:

    enum AtlasStaticWalkingConState {
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
      Idle,                 // 12
      NUM_CON_STATE
    };

    // controls _state -> idle mode
    bool _inited;
    double _stateTime;
    int _state;
    int _contactState;
    int _stepCount;
    
    // need to think about how to lock this thing.
    // world coordinate foot steps
    std::list <SFootStep> _footSteps;
    Eigen::Vector3d _lastFeet[LR];
    bool _isFSSafe;

    bool _com_int_flag[LR];
    double _com_d_over_foot[2];
    double _com_d_i[LR][2];
    double _com_d_i_ini[LR][2];
    double _foot_d_i[LR][2];

    double _realFoot_d[2][3];
    double _cop_comp_z;

    /////////////////////////////////////////
    // knots for interpolation
    int _shiftCOMstart;
    int _shiftCOMend;
    knot_t _com0[3], _com1[3];
    knot_t _wl0, _wl1;
    knot_t _swgFoot0[3], _swgFoot1[3];
    Eigen::Quaterniond _pelvQ0, _pelvQ1;
    Eigen::Quaterniond _torsoQ0, _torsoQ1;
    Eigen::Quaterniond _swgQ0, _swgQ1;
    int _swingFoot;
    int _stanceFoot;
    int _swingDir;
    int _toeOffFoot;
    SFootStep _nxtFootStep;
    double _liftoffDuration;
    
    // toe off stuff
    bool _doToeOff;
    knot_t _toeOffPitch0, _toeOffPitch1;
    Eigen::Vector3d _toeOffAxis;

    bool _swingLegJointMode;
    /////////////////////////////////////////

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
    bool procNextStep(const RobotState &rs);

    void fillDesireds(const RobotState &rs, const TrajPoint<12,0> &root, const TrajPoint<7,0> foot[LR]);
    void gotoState(const RobotState &rs, AtlasStaticWalkingConState nxt);

    WalkingCon::FootStage _footStage[LR];

    const static std::string conModeNames[NUM_CON_STATE];
    Eigen::Vector2d w_cop2b_cop(const RobotState &rs, int side);

  public:
    ///////////////////////////////////////////////////////
    // params
    double com_z;
    double liftoff_rate;
    double liftoff_height;
    double swing_rate;
    double ds_duration;
    double ss_duration;
    double ss_lower_z_vel;
    double ss_height;

    //double com_on_foot_offset[LR][2];

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

    AtlasStaticWalkingCon();
    ~AtlasStaticWalkingCon();
    
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

    inline bool canModifyFootStep() const { return _isFSSafe; }
    inline void appendFootStep(const SFootStep &fs) { _footSteps.push_back(fs); }
    void deleteFootStep(const SFootStep &fs, int i);
    inline const std::list <SFootStep> &getFootSteps() const { return _footSteps; }

    void addToLog(BatchLogger &logger);

    ////////////////////////////////////////////
    // not implemented
    inline const double *getPlannedCurRoot(const RobotState &rs) const { return NULL; }
    inline const double *getPlannedCurLeftFoot(const RobotState &rs) const { return NULL; }
    inline const double *getPlannedCurRightFoot(const RobotState &rs) const { return NULL; }
};

#endif
