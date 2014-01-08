#ifndef __CMU_CTRL_UTILS_H
#define __CMU_CTRL_UTILS_H

//#include <../AtlasRobotInterface/include/AtlasInterface.h>
#include <atlas_msgs/AtlasCommand.h>
#include <atlas_msgs/AtlasState.h>
#include <cmu_walk/RobotState.h>
#include <cmu_walk/Command.h>
#include <cmu_walk/kinematicfilter.h>
#include <cmu_walk/Logger.h> 
#include <cmu_walk/Utils.hpp>
#include <cmu_walk/RobotState.h>
#include <cmu_walk/Command.h>
#include <cmu_walk/KFdata.h>
#include <cmu_walk/WalkingCon.h>
#include <cmu_walk/Butter1.h>
#include <atlas_ros_msgs/sf_state_est.h>

#define TIME_STEP     0.001

class CMUCtrlUtils 
{ 
  public:
    enum CTRL_MODE {
      FF = 0,
      PD,
      PD_FF,
    };

    enum FF_CONST_INT_MODE {
      NONE = 0,
      TORQUE,
      POSITION
    };
   
    static const CTRL_MODE default_f_mask[N_JOINTS];
    static const FF_CONST_INT_MODE default_ff_const_mask[N_JOINTS];
    
    // acuator side gains 
    static const double default_act_f_gains[N_JOINTS];
    static const double default_act_ff_qd_gains[N_JOINTS];
    static const double default_q_gains[N_JOINTS];
    static const double default_qd_gains[N_JOINTS];
    static const double default_f_gains[N_JOINTS];
    static const double default_ff_qd_gains[N_JOINTS];
    static const double default_ff_const[N_JOINTS];
   
    static const double default_q[N_JOINTS];

    // has to be static to make simulator interface to work
    CTRL_MODE f_mask[N_JOINTS];
    
    FF_CONST_INT_MODE ff_const_mask[N_JOINTS];
    
    double act_f_gains[N_JOINTS];
    double act_ff_qd_gains[N_JOINTS];
    double q_gains[N_JOINTS];
    double qd_gains[N_JOINTS];
    double f_gains[N_JOINTS];
    double ff_qd_gains[N_JOINTS];
    double ff_const[N_JOINTS];

    ////////////////////////////////////////////////////////////////
    // raw data from robot
    double initTime;
    double time;
    double rootq[4];
    double root[6];
    double rootd[6];
    double root_b_w[3];
    double foot_forces[LR][6];
    double hand_forces[LR][6];
    double raw_hand_forces[LR][6];
    
    // KF measurement
    double foot_pos_m[LR][3];
    double foot_vel_m[LR][3];

    double joints[N_JOINTS];
    double jointsd[N_JOINTS]; 
    double joints_torque[N_JOINTS]; 

    double imu_angular_velocity[3];
    double imu_linear_acceleration[3];
    double imu_orientation[4]; 
    
    double bdi_root_est[3];
    double bdi_rootd_est[3];
    double bdi_foot_est[LR][3];

    // filter
    double imu_angular_velocity_f[3];
    double imu_linear_acceleration_f[3];
    Eigen::Quaterniond imu_orientation_raw;

    Butter1 *jointsd_filter[N_JOINTS];

    ////////////////////////////////////////////////////////////////
    
    CMUCtrlUtils();
    ~CMUCtrlUtils();

    // for robot
    //void PackDataToRobot(const Command &cmd, double t, AtlasControlDataToRobot &dtr) { PackDataToRobot(cmd.trq_ff, cmd.joints_d, cmd.jointsd_d, dtr, t); }
    //void PackDataToRobot(const double torque_d[N_JOINTS], const double theta_d[N_JOINTS], const double thetad_d[N_JOINTS], AtlasControlDataToRobot &dtr, double commandTime=-1);
    //void UnpackDataFromRobot(const AtlasControlDataFromRobot &dfr, double yaw = 0);

    // for simulator
    void PackDataToRobot(const Command &cmd, double t, atlas_msgs::AtlasCommand &dtr) { PackDataToRobot(cmd.trq_ff, cmd.joints_d, cmd.jointsd_d, dtr, t); }
    void PackDataToRobot(const double torque_d[N_JOINTS], const double theta_d[N_JOINTS], const double thetad_d[N_JOINTS], atlas_msgs::AtlasCommand &dtr, double commandTime=-1);
    void UnpackDataFromRobot(const atlas_msgs::AtlasState &dfr, double yaw = 0);
    
    void estimateStateDummy();
    void estimateState(int cState, KinematicFilter3 &kcekf, double l_foot_fz, double r_foot_fz); // Siyuan walking
    void estimateStateEric(int cState, KinematicFilterEric &kcekf, double l_foot_fz, double r_foot_fz); //Eric ladder
    bool updateRobotState(int contactState, RobotState &rs);
    void init_KF(KinematicFilter3 &kcekf, RobotState::ModelType type, double z);
    void init_KFEric(KinematicFilterEric &kcekf, RobotState::ModelType type, double z);
    
    void setSpineMode(CTRL_MODE mode) { for(int i = 0; i < 3; i++)    f_mask[i] = mode; }
    void setLegMode(CTRL_MODE mode)   { for(int i = 4; i < 16; i++)   f_mask[i] = mode; }
    void setArmMode(CTRL_MODE mode)   { for(int i = 16; i < 28; i++)  f_mask[i] = mode; }
     
    inline void restore_f_mask(int i)        { f_mask[i] = default_f_mask[i]; }
    inline void restore_ff_const_mask(int i) { ff_const_mask[i] = default_ff_const_mask[i]; } 
    inline void restore_q_gains(int i)       { q_gains[i] = default_q_gains[i]; }
    inline void restore_qd_gains(int i)      { qd_gains[i] = default_qd_gains[i]; }
    inline void restore_f_gains(int i)       { f_gains[i] = default_f_gains[i]; }
    inline void restore_ff_qd_gains(int i)   { ff_qd_gains[i] = default_ff_qd_gains[i]; }
    inline void restore_ff_const(int i)      { ff_const[i] = default_ff_const[i]; }
    inline void restore_act_f_gains(int i)   { act_f_gains[i] = default_act_f_gains[i]; }
    inline void restore_act_ff_qd_gains(int i) { act_ff_qd_gains[i] = default_act_ff_qd_gains[i]; }
    
    inline void restore_f_mask()             { memcpy(f_mask, default_f_mask, N_JOINTS*sizeof(CTRL_MODE)); } 
    inline void restore_ff_const_mask()      { memcpy(ff_const_mask, default_ff_const_mask, N_JOINTS*sizeof(FF_CONST_INT_MODE)); } 
    inline void restore_q_gains()            { dvec_copy(q_gains, default_q_gains, N_JOINTS); }  
    inline void restore_qd_gains()           { dvec_copy(qd_gains, default_qd_gains, N_JOINTS); }  
    inline void restore_f_gains()            { dvec_copy(f_gains, default_f_gains, N_JOINTS); }  
    inline void restore_ff_qd_gains()        { dvec_copy(ff_qd_gains, default_ff_qd_gains, N_JOINTS); }  
    inline void restore_ff_const()           { dvec_copy(ff_const, default_ff_const, N_JOINTS); }  
    inline void restore_act_f_gains()        { dvec_copy(act_f_gains, default_act_f_gains, N_JOINTS); }  
    inline void restore_act_ff_qd_gains()    { dvec_copy(act_ff_qd_gains, default_act_ff_qd_gains, N_JOINTS); }  
    
    void restoreAllGains();
    //void setupForceGains(AtlasControlDataToRobot &dtr, int i);
    //void setupPDGains(AtlasControlDataToRobot &dtr, int i);
    //void setupZeroGains(AtlasControlDataToRobot &dtr, int i);
    void setupForceGains(atlas_msgs::AtlasCommand &dtr, int i);
    void setupPDGains(atlas_msgs::AtlasCommand &dtr, int i);
    void setupZeroGains(atlas_msgs::AtlasCommand &dtr, int i);

    // integrators on ffconst
    void integrate_ff_const_trq_lpf(const double trq_d[N_JOINTS], const double trq[N_JOINTS], double gain);
    void integrate_ff_const_trq(double trq_d, double trq, double gain, int idx);
    void integrate_ff_const_pos(double pos_d, double pos, double gain, int idx);
    

    void addToLog(BatchLogger &logger);
    
};

bool isPrevCommandSet(double currentTime);
double *getPrevCommand();

void packPoseOut(const RobotState &rs, atlas_ros_msgs::sf_state_est &pose_out);

void load_KFParams(const std::string &pkg_name, KinematicFilter3 &kcekf);
void load_KFEricParams(const std::string &pkg_name, KinematicFilterEric &kcekf);
void load_sf_params(const std::string &pkg_name, const std::string &idName, const std::string &ikName, const std::string &wcName, WalkingCon &wc);

#endif
