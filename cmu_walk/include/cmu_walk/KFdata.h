#ifndef DATA_H
#define DATA_H 
#include "drc_common_defines.h"
#include "drc_pelvis_defines.h"
//#include "../../atlas.h"  

typedef struct {
    double time_step; 
    double current_time;
    double last_action_time;
    double last_print_time;
    double state;
    
    // BDI stuff from ros
    double bdi_pos[XYZ];
    double bdi_vel[XYZ];
    double bdi_feet[LR][7];

    // Actual States from Gazebo
    double root_pos[XYZ];
    double root_q[QQ];
    double root_vel[XYZ];
    double root_w[XYZ];
    double joints[N_JOINTS];
    double jointsd[N_JOINTS];
    double torques[N_JOINTS];
    double foot_pos[LR][XYZ]; // Foot position
    double foot_vel[LR][XYZ]; // Foot velocity
    double root_to_foot[LR][XYZ]; // Root relative to foot
    
    // Sensor data
    // IMU
    double imu_orientation[4];
    double imu_angular_velocity[3];
    double imu_linear_acceleration[3];
    double imu_linear_acc_world[3]; // world frame linear acc 
    // Force torque sensors
    double lcontact_force[3]; // Left foot contact force
    double lcontact_torque[3]; // left foot contact torque
    double rcontact_force[3]; // right foot contact force
    double rcontact_torque[3]; //right foot contact torque
    
    // Filtered states
    // JointEKF filters
    double KFjointsd[N_JOINTS];
    // IMU filter
    double KFroot_pos[XYZ];
    double KFroot_q[QQ];
    double KFroot_vel[XYZ];
    double KFroot_w[XYZ];
    double KFroot_acc[XYZ];//World acceleration
    double KFroot_acc_pelv[XYZ];// Local angular velocity
    double KFw_b[XYZ]; // Angular bias
    double KFa_b[XYZ]; // Acc bias
    double KFfoot_pos[LR][XYZ]; // Foot position
    double KFfoot_vel[LR][XYZ]; // Foot velocity
    double KFroot_to_foot[LR][XYZ]; // Root relative to foot
    double KFroot_to_foot_pelv[XYZ]; // Root relative to foot in pelvis frame
    double KFroot_q_from_kc[QQ];
    
    /*double lcontact_force_LPF[3]; // Left foot contact force*/
    /*double lcontact_torque_LPF[3]; // left foot contact torque*/
    /*double rcontact_force_LPF[3]; // right foot contact force*/
    /*double rcontact_torque_LPF[3]; //right foot contact torque*/
    /*double lcontact_force_ave[3]; // Left foot contact force*/
    /*double lcontact_torque_ave[3]; // left foot contact torque*/
    /*double rcontact_force_ave[3]; // right foot contact force*/
    /*double rcontact_torque_ave[3]; //right foot contact torque*/
    // double KFStateQuat[PELV_N_SDFAST_STATE]; // KF state
    // double KFPosition[3]; // Pelvis position
    // double KFVelocity[3]; // Pelvis velocity
    
    
    double imu_theta;
    double kf_theta;
    double cs_from_fz; // Contact state from Fz
    double mag1; // Value for innovation normal on velocity
    double mag2;
    
}KFdata;                                                          

//void init_data(KFdata* s);
//bool save_data();
//bool write_the_mrdplot_file(KFdata* s);
#endif /* DATA_H */
