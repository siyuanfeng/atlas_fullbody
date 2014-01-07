#include <test_walk/cmu_ctrl_utils.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <cmu_walk/Eigen_utils.hpp>
#include <cmu_walk/Utils.hpp>
#include <cmu_walk/Transmission.h>

#define TIME_STEP     0.001

#define ACUATOR_SIDE_GAINS

//for transitioning between EW and SF
double prevTheta_d[28];
bool prevCommandSet = false;
double timeOfCommandSet;

bool isPrevCommandSet(double currentTime) {
	return prevCommandSet && (currentTime-timeOfCommandSet < .25);
}
double *getPrevCommand() {
	return prevTheta_d;
}

// acuator side gain
// LBZ, ankle are not controlled this way
const double CMUCtrlUtils::default_act_f_gains[N_JOINTS] = {
  0, 1.5, 1,
  0,
  5, 1, 1, 1, 0, 0,
  5, 1, 1, 1, 0, 0,
  12, 30, 15, 15, 15, 15,
  12, 30, 15, 15, 15, 15};

const double CMUCtrlUtils::default_act_ff_qd_gains[N_JOINTS] = {
  0, 35, 20,
  0,
  35, 35, 35, 35, 0, 0,
  35, 35, 35, 35, 0, 0,
  60, 80, 50, 50, 30, 50,
  60, 80, 50, 50, 30, 50};

const double CMUCtrlUtils::default_q_gains[N_JOINTS] = {
  15, 60, 60,
  5,
  60.0, 60.0, 60.0, 60.0, 2000.0, 2000.0,
  60.0, 60.0, 60.0, 60.0, 2000.0, 2000.0,
  10.0, 10.0, 18.0, 18.0, 8.0, 8.0,
  10.0, 10.0, 18.0, 18.0, 8.0, 8.0};

const double CMUCtrlUtils::default_qd_gains[N_JOINTS] = { 
  0.75/2.0, 0.75/2.0, 0.75/2.0,
  0.75/2.0,
  0.425/2., 0.425/2.0, 0.425/2.0, 0.425/2.0, 4.25/2.0, 4.25/2.0,
  0.425/2., 0.425/2.0, 0.425/2.0, 0.425/2.0, 4.25/2.0, 4.25/2.0,
  0.60/2.0, 0.60/2.0, 0.60/2.0, 0.60/2.0, 0.10/2.0, 0.10/2.0,
  0.60/2.0, 0.60/2.0, 0.60/2.0, 0.60/2.0, 0.10/2.0, 0.10/2.0 };

// LBZ, ankle are using these force gains
const double CMUCtrlUtils::default_f_gains[N_JOINTS] = {
  0.02,  0.02,  0.03,
  0.012,
  0.05,  0.03,  0.04,  0.05,  1.7,  1.7,
  0.05,  0.02,  0.04,  0.04,  1.7,  1.7,
  0.02,  0.02,  0.02,  0.02,  0.02,  0.02,
  0.02,  0.02,  0.02,  0.02,  0.02,  0.02};

const double CMUCtrlUtils::default_ff_qd_gains[N_JOINTS] = {
  0.3, 1, 1,
  0,
  0.8, 2, 2.0, 1.5, 24, 24,
  0.8, 2, 2.0, 1.5, 24, 24,
  0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0};

const double CMUCtrlUtils::default_ff_const[N_JOINTS] = {
  0.0, 0.05, 0.16,
  0,
  0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0,
  //-0.4, 0.2, 0.12, -0.076, 0.028, 0.038,
  //0.2, -0.12, -0.032, -0.135, 0.10, 0.086,  // for more position
  //0.2, -0.12, -0.032, -0.135, 0.10, 0.86, // for trq tracking
  0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0}; 

// k_q_p * ( q_d - q ) + 
// k_q_i * 1/s * ( q_d - q ) + 
// k_qd_p * ( qd_d - qd ) + 
// k_f_p * ( f_d - f ) + 
// ff_qd * qd + 
// ff_qd_d * qd_d + 
// ff_f_d * f_d + 
// ff_const
const CMUCtrlUtils::CTRL_MODE CMUCtrlUtils::default_f_mask[N_JOINTS] = {
  PD_FF, PD_FF, PD_FF,
  PD,
  PD_FF, PD_FF, PD_FF, PD_FF, PD_FF, PD_FF,
  PD_FF, PD_FF, PD_FF, PD_FF, PD_FF, PD_FF,  
  PD, PD, PD, PD, PD, PD,
  PD, PD, PD, PD, PD, PD};

const CMUCtrlUtils::FF_CONST_INT_MODE CMUCtrlUtils::default_ff_const_mask[N_JOINTS] = {
  NONE, NONE, NONE, 
  NONE, 
  NONE, NONE, NONE, NONE, NONE, NONE, 
  NONE, NONE, NONE, NONE, NONE, NONE, 
  NONE, NONE, NONE, NONE, NONE, NONE, 
  NONE, NONE, NONE, NONE, NONE, NONE}; 

CMUCtrlUtils::CMUCtrlUtils()
{
  initTime = -1;

  for (int i = 0; i < 4; i++) {
    rootq[i] = 0;
    imu_orientation[i] = 0;
  }
  imu_orientation[3] = rootq[3] = 1;

  for (int i = 0; i < 6; i++) {
    root[i] = 0;
    rootd[i] = 0;
    for (int side = 0; side < LR; side++) 
      foot_forces[side][i] = 0;
  }
  root[2] = 0.9545;

  for (int i = 0; i < 6; i++)
    rootd[i] = 0;

  for (int i = 0; i < 3; i++) {
    root_b_w[i] = 0;
    imu_angular_velocity[i] = 0;
    imu_linear_acceleration[i] = 0;
  }

  for (int i = 0; i < N_JOINTS; i++) {
    joints[i] = 0;
    jointsd[i] = 0;
    joints_torque[i] = 0;

    jointsd_filter[i] = new Butter1(0.02);
  }
  restore_f_mask();
  restore_ff_const_mask();

  restoreAllGains();
}

CMUCtrlUtils::~CMUCtrlUtils()
{
  for (int i = 0; i < N_JOINTS; i++) 
    delete jointsd_filter[i];
}

void CMUCtrlUtils::restoreAllGains()
{ 
  restore_q_gains();
  restore_qd_gains();
  restore_f_gains();
  restore_ff_qd_gains();
  restore_ff_const(); 
  restore_act_f_gains();
  restore_act_ff_qd_gains(); 
}


void CMUCtrlUtils::setupForceGains(atlas_msgs::AtlasCommand &dtr, int i)
{
  // simulator gives perfect torque, so we don't need to do anything
  // here.
}

void CMUCtrlUtils::setupPDGains(atlas_msgs::AtlasCommand &dtr, int i)
{
  dtr.kp_position[i] = q_gains[i];
  dtr.kd_position[i] = qd_gains[i]; 
}

void CMUCtrlUtils::setupZeroGains(atlas_msgs::AtlasCommand &dtr, int i) 
{
  dtr.kp_position[i] = 0;
  dtr.ki_position[i] = 0;
  dtr.kd_position[i] = 0;
  dtr.kp_velocity[i] = 0;
}

void CMUCtrlUtils::PackDataToRobot(const double torque_d[N_JOINTS], const double theta_d[N_JOINTS], const double thetad_d[N_JOINTS], atlas_msgs::AtlasCommand &dtr, double commandTime)
{
  // https://bitbucket.org/osrf/drcsim/src/7ff63e8aab47973b10539420d5a8685aa2bb9384/ros/atlas_msgs/msg/AtlasCommand.msg?at=default
  // init the arrays
  if (dtr.position.size() != N_JOINTS) {
    dtr.position.resize(N_JOINTS);
    dtr.velocity.resize(N_JOINTS);
    dtr.effort.resize(N_JOINTS);
    dtr.kp_position.resize(N_JOINTS);
    dtr.ki_position.resize(N_JOINTS);
    dtr.kd_position.resize(N_JOINTS);
    dtr.kp_velocity.resize(N_JOINTS);
    dtr.i_effort_min.resize(N_JOINTS);
    dtr.i_effort_max.resize(N_JOINTS);
    dtr.k_effort.resize(N_JOINTS);
  }

  for (int i = 0; i < N_JOINTS; i++) {
    dtr.position[i] = theta_d[i];
    dtr.velocity[i] = thetad_d[i];
    dtr.effort[i] = torque_d[i];

    // setup gains
    setupZeroGains(dtr, i);
    switch (f_mask[i]) {
      case FF:
        setupForceGains(dtr, i);
        break;

      case PD:
        setupPDGains(dtr, i);
        break;
      
      case PD_FF:
        setupForceGains(dtr, i);
        setupPDGains(dtr, i);
        break;
    }
    
    dtr.i_effort_min[i] = 0;
    dtr.i_effort_max[i] = 0;
    dtr.k_effort[i] = 255;      // user mode
  }
  
  // sync timeout length, gazebo will wait for 15 ms before doing
  // the next simulation step
  dtr.desired_controller_period_ms = 15;
}

void CMUCtrlUtils::UnpackDataFromRobot(const atlas_msgs::AtlasState &dfr, double yaw)
{
  // https://bitbucket.org/osrf/drcsim/src/d9183cb065bb2c658c63d6d237de313943f4aaa2/ros/atlas_msgs/msg/AtlasState.msg?at=default
  time = dfr.header.stamp.toSec();
  
  if (initTime == -1)
    initTime = time;

  // joints
  for (size_t i = 0; i < N_JOINTS; i++) {
    joints[i] = dfr.position[i];
    jointsd[i] = dfr.velocity[i];
    joints_torque[i] = dfr.effort[i];
  }

  // imu
  imu_angular_velocity[0] = dfr.angular_velocity.x;
  imu_angular_velocity[1] = dfr.angular_velocity.y;
  imu_angular_velocity[2] = dfr.angular_velocity.z;
  imu_linear_acceleration[0] = dfr.linear_acceleration.x;
  imu_linear_acceleration[1] = dfr.linear_acceleration.y;
  imu_linear_acceleration[2] = dfr.linear_acceleration.z; 
  
  imu_orientation_raw = Eigen::Quaterniond(dfr.orientation.w, dfr.orientation.x, dfr.orientation.y, dfr.orientation.z);

  Eigen::Matrix3d rot = imu_orientation_raw.toRotationMatrix();
  rot = Eigen::AngleAxisd(-yaw, Eigen::Vector3d::UnitZ()) * rot;
  Eigen::Map <Eigen::Quaterniond> q(imu_orientation);
  q = Eigen::Quaterniond(rot);

  // foot force
  foot_forces[LEFT][XX] = dfr.l_foot.force.x;
  foot_forces[LEFT][YY] = dfr.l_foot.force.y;
  foot_forces[LEFT][ZZ] = dfr.l_foot.force.z;
  foot_forces[LEFT][XX+3] = dfr.l_foot.torque.x;
  foot_forces[LEFT][YY+3] = dfr.l_foot.torque.y;
  foot_forces[LEFT][ZZ+3] = dfr.l_foot.torque.z;
  foot_forces[RIGHT][XX] = dfr.r_foot.force.x;
  foot_forces[RIGHT][YY] = dfr.r_foot.force.y;
  foot_forces[RIGHT][ZZ] = dfr.r_foot.force.z;
  foot_forces[RIGHT][XX+3] = dfr.r_foot.torque.x;
  foot_forces[RIGHT][YY+3] = dfr.r_foot.torque.y;
  foot_forces[RIGHT][ZZ+3] = dfr.r_foot.torque.z;  
  
  // avoid division by zero when computing cop (Mx / Fz)
  for (int side = LEFT; side < LR; side++) {
    if (fabs(foot_forces[side][ZZ]) < 1)
      foot_forces[side][ZZ] = 1;
  }

  // hand force
  hand_forces[LEFT][XX] = dfr.l_hand.force.x;
  hand_forces[LEFT][YY] = dfr.l_hand.force.y;
  hand_forces[LEFT][ZZ] = dfr.l_hand.force.z;
  hand_forces[LEFT][XX+3] = dfr.l_hand.torque.x;
  hand_forces[LEFT][YY+3] = dfr.l_hand.torque.y;
  hand_forces[LEFT][ZZ+3] = dfr.l_hand.torque.z;
  hand_forces[RIGHT][XX] = dfr.r_hand.force.x;
  hand_forces[RIGHT][YY] = dfr.r_hand.force.y;
  hand_forces[RIGHT][ZZ] = dfr.r_hand.force.z;
  hand_forces[RIGHT][XX+3] = dfr.r_hand.torque.x;
  hand_forces[RIGHT][YY+3] = dfr.r_hand.torque.y;
  hand_forces[RIGHT][ZZ+3] = dfr.r_hand.torque.z;   
}


// Siyuan walking
void CMUCtrlUtils::estimateState(int cState, KinematicFilter3 &kcekf, double l_foot_fz, double r_foot_fz)
{
  // lp filter joints vel
  static double jointsd_h[N_JOINTS] = {0};
  for (int i = 0; i < N_JOINTS; i++) {
    jointsd[i] = low_pass_filter(jointsd_h[i], jointsd[i], 0.9);
    if (i == A_L_LEG_UAY)
      jointsd[i] = low_pass_filter(jointsd_h[i], jointsd[i], 0.99);
    if (i == A_L_LEG_LAX)
      jointsd[i] = low_pass_filter(jointsd_h[i], jointsd[i], 0.99);
    if (i == A_R_LEG_UAY)
      jointsd[i] = low_pass_filter(jointsd_h[i], jointsd[i], 0.99);
    if (i == A_R_LEG_LAX)
      jointsd[i] = low_pass_filter(jointsd_h[i], jointsd[i], 0.99);
  }
  jointsd[A_BACK_UBX] = jointsd_filter[A_BACK_UBX]->deriv(joints[A_BACK_UBX], 333.0);
  /*
  for (int i = 0; i < N_JOINTS; i++) {
    jointsd[i] = jointsd_filter[i]->deriv(joints[i], 333.0);
  }
  */

  // lp filter angular vel
  static double w_h[3] = {0};
  for (int i = 0; i < 3; i++) {
    imu_angular_velocity_f[i] = low_pass_filter(w_h[i], imu_angular_velocity[i], 0.95);
  }
  
  // lp filter angular vel
  static double la_h[3] = {0};
  for (int i = 0; i < 3; i++) {
    imu_linear_acceleration_f[i] = low_pass_filter(la_h[i], imu_linear_acceleration[i], 0.95);
  }
  
  kcekf.makeInputs(imu_orientation, imu_angular_velocity_f, imu_linear_acceleration_f, joints, jointsd);
  kcekf.getContactState(cState);

  kcekf.makeMeasurement(l_foot_fz, r_foot_fz);
  kcekf.filterOneTimeStep_ss();

  // Reset states

  //imuekf.getQuaternion(rootq);    // Reset quaternion
  // Use IMU generated Orientation
  dvec_copy(rootq, imu_orientation, 4);
  dvec_copy(root_b_w, imu_angular_velocity_f, 3);
  
  kcekf.getPosition(root);    // Reset root position
  kcekf.getLinearV(rootd);    // Reset root velocity
  kcekf.getFootPose(LEFT, foot_pos_m[LEFT]);
  kcekf.getFootPose(RIGHT, foot_pos_m[RIGHT]);

  static double rootd_h[3] = {0};
  for (int i = 0; i < 3; i++) {
    rootd[i] = low_pass_filter(rootd_h[i], rootd[i], 0.95);
  }
}

// Eric ladder
void CMUCtrlUtils::estimateStateEric(int cState, KinematicFilterEric &kcekf, double l_foot_fz, double r_foot_fz)
{
  // lp filter joints vel
  static double jointsd_h[N_JOINTS] = {0};
  for (int i = 0; i < N_JOINTS; i++) {
    jointsd[i] = low_pass_filter(jointsd_h[i], jointsd[i], 0.9);
    if (i == A_L_LEG_UAY)
      jointsd[i] = low_pass_filter(jointsd_h[i], jointsd[i], 0.97);
    if (i == A_L_LEG_LAX)
      jointsd[i] = low_pass_filter(jointsd_h[i], jointsd[i], 0.97);
    if (i == A_R_LEG_UAY)
      jointsd[i] = low_pass_filter(jointsd_h[i], jointsd[i], 0.97);
    if (i == A_R_LEG_LAX)
      jointsd[i] = low_pass_filter(jointsd_h[i], jointsd[i], 0.97);
  }
  jointsd[A_BACK_UBX] = jointsd_filter[A_BACK_UBX]->deriv(joints[A_BACK_UBX], 333.0);

  // lp filter angular vel
  static double w_h[3] = {0};
  for (int i = 0; i < 3; i++) {
    imu_angular_velocity_f[i] = low_pass_filter(w_h[i], imu_angular_velocity[i], 0.95);
  }
  
  // lp filter angular vel
  static double la_h[3] = {0};
  for (int i = 0; i < 3; i++) {
    imu_linear_acceleration_f[i] = low_pass_filter(la_h[i], imu_linear_acceleration[i], 0.95);
  }
  
  kcekf.makeInputs(imu_orientation, imu_angular_velocity_f, imu_linear_acceleration_f, joints, jointsd);
  kcekf.getContactState(cState);

  kcekf.makeMeasurement(l_foot_fz, r_foot_fz);
  kcekf.filterOneTimeStep_ss();

  // Reset states

  //imuekf.getQuaternion(rootq);    // Reset quaternion
  // Use IMU generated Orientation
  dvec_copy(rootq, imu_orientation, 4);
  dvec_copy(root_b_w, imu_angular_velocity_f, 3);
  
  kcekf.getPosition(root);    // Reset root position
  kcekf.getLinearV(rootd);    // Reset root velocity
  kcekf.getFootPose(LEFT, foot_pos_m[LEFT]);
  kcekf.getFootPose(RIGHT, foot_pos_m[RIGHT]);

  static double rootd_h[3] = {0};
  for (int i = 0; i < 3; i++) {
    rootd[i] = low_pass_filter(rootd_h[i], rootd[i], 0.95);
  }
}

void CMUCtrlUtils::estimateStateDummy()
{
  dvec_set(root, 0, 6);
  dvec_set(rootd, 0, 6);

  dvec_copy(rootq, imu_orientation, 4);
  dvec_copy(root_b_w, imu_angular_velocity, 3);
}

bool CMUCtrlUtils::updateRobotState(int contactState, RobotState &rs)
{
  rs.ctrl_dt = time - rs.time;
  rs.time = time;
  rs.timeStep = TIME_STEP;
  
  rs.computeSDFvars(root, rootq, rootd, root_b_w, joints, jointsd);
  for(int s = 0; s < 2; s++) {
	  for(int k = 0; k < 2; k++) {
		  Eigen::Vector3d rawHand(raw_hand_forces[s][XX+3*k], raw_hand_forces[s][YY+3*k], raw_hand_forces[s][ZZ+3*k]);
		  Eigen::Vector3d rotHand = rs.handQ[s]*rawHand;
		  for(int i = 0; i < 3; i++)	hand_forces[s][i+3*k] = rotHand[i];
	  }
  }


  // LATER
  //rs.computeSDFvarsFilt(root, rootq, rootd, root_b_w, joints, jointsd, 0.01);
  dvec_copy(rs.joints_torque, joints_torque, N_JOINTS); 

  // copy wrist sensor
  dvec_copy(rs.handForces[LEFT], hand_forces[LEFT], 6);
  dvec_copy(rs.handForces[RIGHT], hand_forces[RIGHT], 6);
  dvec_copy(rs.handForcesRaw[LEFT], raw_hand_forces[LEFT], 6);
  dvec_copy(rs.handForcesRaw[RIGHT], raw_hand_forces[RIGHT], 6);

  // copy force sensor stuff
  for (int i = 0; i < LR; i++) {
    dvec_copy(rs.feet[i].b_F, foot_forces[i], 6);
    // avoid division by zero
    rs.feet[i].b_cop[XX] = -rs.feet[i].b_F[3+YY] / rs.feet[i].b_F[ZZ];
    rs.feet[i].b_cop[YY] = rs.feet[i].b_F[3+XX] / rs.feet[i].b_F[ZZ];

    rs.feet[i].w_F[ZZ] = foot_forces[i][ZZ];  
    // rotate mx my to world frame to get world cop
    Eigen::Vector2d b_m(rs.feet[i].b_F+3);
    Eigen::Map <Eigen::Vector2d> w_m(rs.feet[i].w_F+3);
    w_m = rotate2d(b_m, getYaw(rs.feet[i].w_q));

    rs.feet[i].w_cop[XX] = -rs.feet[i].w_F[3+YY] / rs.feet[i].w_F[ZZ] + rs.feet[i].w_sensor_pos[XX];
    rs.feet[i].w_cop[YY] =  rs.feet[i].w_F[3+XX] / rs.feet[i].w_F[ZZ] + rs.feet[i].w_sensor_pos[YY];
  }
  
  // cop
  rs.cop[XX] = (rs.feet[LEFT].w_cop[XX] * rs.feet[LEFT].w_F[ZZ] + rs.feet[RIGHT].w_cop[XX] * rs.feet[RIGHT].w_F[ZZ]) / (rs.feet[LEFT].w_F[ZZ] + rs.feet[RIGHT].w_F[ZZ]);
  rs.cop[YY] = (rs.feet[LEFT].w_cop[YY] * rs.feet[LEFT].w_F[ZZ] + rs.feet[RIGHT].w_cop[YY] * rs.feet[RIGHT].w_F[ZZ]) / (rs.feet[LEFT].w_F[ZZ] + rs.feet[RIGHT].w_F[ZZ]);

  // contact state based on planned contact state,
  // could come from Ben's filter
  rs.contactState = contactState;

  return true; 
}

void load_KFParams(const std::string &pkg_name, KinematicFilter3 &kcekf)
{
  std::string nameQ;
  std::string nameR;
  // Q
  nameQ = ros::package::getPath(pkg_name) + std::string("/config/filter_param/DSc_Q3");
  
  // R
  nameR = ros::package::getPath(pkg_name) + std::string("/config/filter_param/DSc_R3");

  kcekf.readParams(nameQ, nameR); 
  printf("Kinematic KF3 Parameters read\n");
}

void load_KFEricParams(const std::string &pkg_name, KinematicFilterEric &kcekf)
{
  std::string nameQ;
  std::string nameR;
  // Q
  nameQ = ros::package::getPath(pkg_name) + std::string("/config/filter_param/DSc_Q3");
  
  // R
  nameR = ros::package::getPath(pkg_name) + std::string("/config/filter_param/DSc_R3"); 

  kcekf.readParams(nameQ, nameR); 
  printf("KF eric Parameters read\n");
}

// Siyuan walking
void CMUCtrlUtils::init_KF(KinematicFilter3 &kcekf, RobotState::ModelType type, double z)
{
  // This has to be called to populate internal robot state for KF
  kcekf.makeInputs(imu_orientation, imu_angular_velocity_f, imu_linear_acceleration_f, joints, jointsd); 
  // init root stuff
  dvec_copy(rootq, imu_orientation, 4);

  root[0] = 0;
  root[1] = 0;
  root[2] = z;
  root[3] = 0;
  root[4] = 0;
  root[5] = 0;

  for (int i = 0; i < 6; i++)
    rootd[i] = 0;
  for (int i = 0; i < 3; i++)
    root_b_w[i] = 0;       
  
  kcekf.initKF(type, TIME_STEP, root, rootd);
  printf("Kinematic KF3 initialized\n\n\n\n\n\n");
}
// Eric ladder
void CMUCtrlUtils::init_KFEric(KinematicFilterEric &kcekf, RobotState::ModelType type, double z)
{ 
  // This has to be called to populate internal robot state for KF
  kcekf.makeInputs(imu_orientation, imu_angular_velocity_f, imu_linear_acceleration_f, joints, jointsd); 
  // init root stuff
  dvec_copy(rootq, imu_orientation, 4);

  root[0] = 0;
  root[1] = 0;
  root[2] = z;
  root[3] = 0;
  root[4] = 0;
  root[5] = 0;

  for (int i = 0; i < 6; i++)
    rootd[i] = 0;
  for (int i = 0; i < 3; i++)
    root_b_w[i] = 0;          

  kcekf.initKF(type, TIME_STEP, root, rootd);
  printf("Kinematic KF Eric initialized\n\n\n\n\n\n");
}
  


const double CMUCtrlUtils::default_q[N_JOINTS] = {
  0, 0, 0.0, 
  0, 
  0, 0, -0.23, 0.52, -0.28, 0, 
  0, 0, -0.23, 0.52, -0.28, 0., 
  0.3, -1.3, 2, 0.5, 0., 0, 
  0.3, 1.3, 2, -0.5, 0., 0
};

/*
void Command2sf_out(const RobotState &rs, const IKcmd &ikcmd, const Command &cmd, atlas_ros_msgs::sf_controller_out &msg)
{
  for (int i = 0; i < N_JOINTS; i++)
    msg.ik_joints[i] = cmd.joints_d[i];

  dvec_copy(msg.model_com.data(), rs.com, 3);
  dvec_copy(msg.ikcmd_com.data(), ikcmd.com, 3);

  for (int i = 0; i < 3; i++) {
    msg.model_com[i] -= rs.root[i];
    msg.ikcmd_com[i] -= rs.root[i];
  }

  dvec_copy(msg.ik_root.data(), cmd.root_d, 3);
  dvec_copy(msg.ik_pelv_quat.data(), cmd.rootq_d.coeffs().data(), 4);
  dvec_copy(msg.id_b_grf.data(), cmd.b_grf_d, 12);
  dvec_copy(msg.id_foot_b_cop.data(), cmd.b_cop_d[LEFT], 2);
  dvec_copy(msg.id_foot_b_cop.data()+2, cmd.b_cop_d[RIGHT], 2);
}
*/

void CMUCtrlUtils::addToLog(BatchLogger &logger)
{
	logger.add_datapoint("ConType[back_lbz]","-",(const int*)&(f_mask[0]));
	logger.add_datapoint("ConType[back_mby]","-",(const int*)&(f_mask[1]));
	logger.add_datapoint("ConType[back_ubx]","-",(const int*)&(f_mask[2]));
	logger.add_datapoint("ConType[neck_ay]","-",(const int*)&(f_mask[3]));
	logger.add_datapoint("ConType[l_leg_uhz]","-",(const int*)&(f_mask[4]));
	logger.add_datapoint("ConType[l_leg_mhx]","-",(const int*)&(f_mask[5]));
	logger.add_datapoint("ConType[l_leg_lhy]","-",(const int*)&(f_mask[6]));
	logger.add_datapoint("ConType[l_leg_kny]","-",(const int*)&(f_mask[7]));
	logger.add_datapoint("ConType[l_leg_uay]","-",(const int*)&(f_mask[8]));
	logger.add_datapoint("ConType[l_leg_lax]","-",(const int*)&(f_mask[9]));
	logger.add_datapoint("ConType[r_leg_uhz]","-",(const int*)&(f_mask[10]));
	logger.add_datapoint("ConType[r_leg_mhx]","-",(const int*)&(f_mask[11]));
	logger.add_datapoint("ConType[r_leg_lhy]","-",(const int*)&(f_mask[12]));
	logger.add_datapoint("ConType[r_leg_kny]","-",(const int*)&(f_mask[13]));
	logger.add_datapoint("ConType[r_leg_uay]","-",(const int*)&(f_mask[14]));
	logger.add_datapoint("ConType[r_leg_lax]","-",(const int*)&(f_mask[15]));
	logger.add_datapoint("ConType[l_arm_usy]","-",(const int*)&(f_mask[16]));
	logger.add_datapoint("ConType[l_arm_shx]","-",(const int*)&(f_mask[17]));
	logger.add_datapoint("ConType[l_arm_ely]","-",(const int*)&(f_mask[18]));
	logger.add_datapoint("ConType[l_arm_elx]","-",(const int*)&(f_mask[19]));
	logger.add_datapoint("ConType[l_arm_uwy]","-",(const int*)&(f_mask[20]));
	logger.add_datapoint("ConType[l_arm_mwx]","-",(const int*)&(f_mask[21]));
	logger.add_datapoint("ConType[r_arm_usy]","-",(const int*)&(f_mask[22]));
	logger.add_datapoint("ConType[r_arm_shx]","-",(const int*)&(f_mask[23]));
	logger.add_datapoint("ConType[r_arm_ely]","-",(const int*)&(f_mask[24]));
	logger.add_datapoint("ConType[r_arm_elx]","-",(const int*)&(f_mask[25]));
	logger.add_datapoint("ConType[r_arm_uwy]","-",(const int*)&(f_mask[26]));
	logger.add_datapoint("ConType[r_arm_mwx]","-",(const int*)&(f_mask[27]));

  logger.add_datapoint("IMU_raw.quat[x]","m/s/s",&(imu_orientation_raw.coeffs().data()[0]));
  logger.add_datapoint("IMU_raw.quat[y]","m/s/s",&(imu_orientation_raw.coeffs().data()[1]));
  logger.add_datapoint("IMU_raw.quat[z]","m/s/s",&(imu_orientation_raw.coeffs().data()[2]));
  logger.add_datapoint("IMU_raw.quat[w]","m/s/s",&(imu_orientation_raw.coeffs().data()[3]));
  logger.add_quat("IMU_raw", &imu_orientation_raw);

  logger.add_datapoint("IMU.acc[X]","m/s/s",&(imu_linear_acceleration[XX]));
  logger.add_datapoint("IMU.acc[Y]","m/s/s",&(imu_linear_acceleration[YY]));
  logger.add_datapoint("IMU.acc[Z]","m/s/s",&(imu_linear_acceleration[ZZ]));
  logger.add_datapoint("IMU.w[X]","rad/s",&(imu_angular_velocity[XX]));
  logger.add_datapoint("IMU.w[Y]","rad/s",&(imu_angular_velocity[YY]));
  logger.add_datapoint("IMU.w[Z]","rad/s",&(imu_angular_velocity[ZZ])); 
  
  logger.add_datapoint("IMU.acc_f[X]","m/s/s",&(imu_linear_acceleration_f[XX]));
  logger.add_datapoint("IMU.acc_f[Y]","m/s/s",&(imu_linear_acceleration_f[YY]));
  logger.add_datapoint("IMU.acc_f[Z]","m/s/s",&(imu_linear_acceleration_f[ZZ]));
  logger.add_datapoint("IMU.w_f[X]","rad/s",&(imu_angular_velocity_f[XX]));
  logger.add_datapoint("IMU.w_f[Y]","rad/s",&(imu_angular_velocity_f[YY]));
  logger.add_datapoint("IMU.w_f[Z]","rad/s",&(imu_angular_velocity_f[ZZ])); 

  logger.add_datapoint("bdi.root[x]","rad/s",&(bdi_root_est[0])); 
  logger.add_datapoint("bdi.root[y]","rad/s",&(bdi_root_est[1])); 
  logger.add_datapoint("bdi.root[z]","rad/s",&(bdi_root_est[2])); 
  logger.add_datapoint("bdi.rootd[x]","rad/s",&(bdi_rootd_est[0])); 
  logger.add_datapoint("bdi.rootd[y]","rad/s",&(bdi_rootd_est[1])); 
  logger.add_datapoint("bdi.rootd[z]","rad/s",&(bdi_rootd_est[2])); 

  logger.add_datapoint("bdi.foot[L][x]","rad/s",&(bdi_foot_est[LEFT][0])); 
  logger.add_datapoint("bdi.foot[L][y]","rad/s",&(bdi_foot_est[LEFT][1])); 
  logger.add_datapoint("bdi.foot[L][z]","rad/s",&(bdi_foot_est[LEFT][2])); 
  logger.add_datapoint("bdi.foot[R][x]","rad/s",&(bdi_foot_est[RIGHT][0])); 
  logger.add_datapoint("bdi.foot[R][y]","rad/s",&(bdi_foot_est[RIGHT][1])); 
  logger.add_datapoint("bdi.foot[R][z]","rad/s",&(bdi_foot_est[RIGHT][2])); 
  

  char buf[1000];
  for (int i = 0; i < N_JOINTS; i++) {
    sprintf(buf, "%s_ffconst", RobotState::joint_names[i].c_str());
    logger.add_datapoint(buf, "-", &(ff_const[i]));
  }
  // Measurement used in KF
  logger.add_datapoint("kf.observ.lfoot[x]","m",&(foot_pos_m[LEFT][XX])); 
  logger.add_datapoint("kf.observ.lfoot[y]","m",&(foot_pos_m[LEFT][YY])); 
  logger.add_datapoint("kf.observ.lfoot[z]","m",&(foot_pos_m[LEFT][ZZ])); 
  logger.add_datapoint("kf.observ.rfoot[x]","m",&(foot_pos_m[RIGHT][XX])); 
  logger.add_datapoint("kf.observ.rfoot[y]","m",&(foot_pos_m[RIGHT][YY])); 
  logger.add_datapoint("kf.observ.rfoot[z]","m",&(foot_pos_m[RIGHT][ZZ])); 
}

void CMUCtrlUtils::integrate_ff_const_pos(double pos_d, double pos, double gain, int idx) 
{ 
  ff_const[idx] += gain*(pos_d - pos); 
}

void CMUCtrlUtils::integrate_ff_const_trq(double trq_d, double trq, double gain, int idx) 
{
  ff_const[idx] += gain*(trq_d - trq); 
}

void CMUCtrlUtils::integrate_ff_const_trq_lpf(const double trq_d[N_JOINTS], const double trq[N_JOINTS], double gain) 
{
  static double trq_err_h[N_JOINTS] = {0};
  for (int i = 0; i < N_JOINTS; i++) {
    ff_const[i] += gain*low_pass_filter(trq_err_h[i], trq_d[i] - trq[i], 0.9);
  }
}

void load_sf_params(const std::string &pkg_name, const std::string &idName, const std::string &ikName, const std::string &wcName, WalkingCon &wc)
{
  std::string name;
  bool ret;
  std::ifstream in; 
  
  name = ros::package::getPath(pkg_name) + idName;
  in.open(name.c_str());
  ret = wc.idCon->readParams(in);
  in.close();
 
  name = ros::package::getPath(pkg_name) + ikName;
  in.open(name.c_str());
  ret = wc.ikCon->readParams(in);
  in.close();

  name = ros::package::getPath(pkg_name) + wcName;
  in.open(name.c_str());
  ret &= wc.readParams(in);
  in.close();

  assert(ret);
}



 
// k_q_p * ( q_d - q ) + 
// k_q_i * 1/s * ( q_d - q ) + 
// k_qd_p * ( qd_d - qd ) + 
// k_f_p * ( f_d - f ) + 
// ff_qd * qd + 
// ff_qd_d * qd_d + 
// ff_f_d * f_d + 
// ff_const
/*
void CMUCtrlUtils::PackDataToRobot(const double torque_d[N_JOINTS], const double theta_d[N_JOINTS], const double thetad_d[N_JOINTS], AtlasControlDataToRobot &dtr, double commandTime)
{
	prevCommandSet = true;
	if(time !=-1)	timeOfCommandSet = commandTime;
	for(int i = 0; i < 28; i++)		prevTheta_d[i] = theta_d[i];

  for (size_t i = 0; i < N_JOINTS; i++) 
  {
    dtr.j[i].q_d = theta_d[i];
    dtr.j[i].qd_d = thetad_d[i];
    dtr.j[i].f_d = torque_d[i];
    
    //////////////////////////////////////////////////////
    // sanity check 
    if (is_bad_num(dtr.j[i].q_d)) {
      dtr.j[i].q_d = default_q[i];
      fprintf(stderr, "CMUCtrlUtils, setting q_d[%s] = nan.\n", RobotState::joint_names[i].c_str());
    }
    if (is_bad_num(dtr.j[i].qd_d)) {
      dtr.j[i].qd_d = 0;
      fprintf(stderr, "CMUCtrlUtils, setting qd_d[%s] = nan.\n", RobotState::joint_names[i].c_str());
    }
    if (is_bad_num(dtr.j[i].f_d)) {
      dtr.j[i].f_d = 0;
      dtr.j[i].q_d = joints[i];
      dtr.j[i].qd_d = 0;
      f_mask[i] = PD;
      fprintf(stderr, "CMUCtrlUtils, setting f_d[%s] = nan. PD mode on.\n", RobotState::joint_names[i].c_str());
    }
    
    //////////////////////////////////////////////////////
    // integrate ff_const
    if (ff_const_mask[i] == TORQUE) {
      integrate_ff_const_trq(dtr.j[i].f_d, joints_torque[i], 0.0002, i);
    }
    else if (ff_const_mask[i] == POSITION)
      integrate_ff_const_pos(dtr.j[i].q_d, joints[i], 0.03, i);

    clamp(ff_const[i], -4, 4);

    //////////////////////////////////////////////////////
    // setup gains
    setupZeroGains(dtr, i);

    switch (f_mask[i]) {
      case VALVE:
        // this is a hack
        dtr.jparams[i].ff_const = dtr.j[i].f_d;
        break;

      case FF:
        setupForceGains(dtr, i);
        dtr.jparams[i].ff_const = ff_const[i];
        break;

      case PD:
        setupPDGains(dtr, i);
        dtr.jparams[i].ff_const = ff_const[i]; 
        break;
      
      case PD_FF:
        setupForceGains(dtr, i);
        setupPDGains(dtr, i);
        dtr.jparams[i].ff_const = ff_const[i]; 
        break;

      case D_FF:
        setupForceGains(dtr, i);
        dtr.jparams[i].k_qd_p = qd_gains[i]; 
        dtr.jparams[i].ff_const = ff_const[i]; 
        break; 

      case PD_FF_SWING:
        setupForceGains(dtr, i);
        setupPDGains(dtr, i);
        dtr.jparams[i].ff_const = ff_const[i];
        
        dtr.jparams[i].k_f_p /= 2.0;
        dtr.jparams[i].ff_qd /= 20;
        dtr.jparams[i].k_q_p /= 10.0;
        //dtr.jparams[i].k_qd_p /= 7.0;
        break;
    }
  }
}

void CMUCtrlUtils::setupForceGains(AtlasControlDataToRobot &dtr, int i)
{
#ifdef ACUATOR_SIDE_GAINS
  // what ihmc people used
  if (Transmission::isAcuatorSide(i)) {
    double l = Transmission::getMomentArm(i, joints[i]);
    dtr.jparams[i].k_f_p = act_f_gains[i] * l;
    dtr.jparams[i].ff_qd = act_ff_qd_gains[i] * l;
  }
  else {
    dtr.jparams[i].k_f_p = f_gains[i];
    dtr.jparams[i].ff_qd = ff_qd_gains[i];
  }
#else
  dtr.jparams[i].ff_qd = ff_qd_gains[i];
  dtr.jparams[i].k_f_p = f_gains[i];
#endif 
}

void CMUCtrlUtils::setupPDGains(AtlasControlDataToRobot &dtr, int i)
{
  dtr.jparams[i].k_q_p = q_gains[i];
  dtr.jparams[i].k_qd_p = qd_gains[i]; 
}

void CMUCtrlUtils::setupZeroGains(AtlasControlDataToRobot &dtr, int i)
{
  dtr.jparams[i].k_q_p = 0;
  dtr.jparams[i].k_q_i = 0; 
  dtr.jparams[i].k_qd_p = 0;
  dtr.jparams[i].k_f_p = 0;
  dtr.jparams[i].ff_qd = 0;
  dtr.jparams[i].ff_qd_d = 0;
  dtr.jparams[i].ff_f_d = 0;
  dtr.jparams[i].ff_const = 0;
}

void CMUCtrlUtils::UnpackDataFromRobot(const AtlasControlDataFromRobot &dfr, double yaw)
{
  ros::Time t = ros::Time();
  t.fromNSec(dfr.timestamp * 1e3);
  time = t.toSec();

  if (initTime == -1)
    initTime = time;

  //time = time - initTime;

  // joints
  for (size_t i = 0; i < N_JOINTS; i++) {
    joints[i] = dfr.j[i].q;
    jointsd[i] = dfr.j[i].qd;
    joints_torque[i] = dfr.j[i].f;
  }

  // imu
  for (size_t i = 0; i < 3; i++) {
    imu_angular_velocity[i] = dfr.filtered_imu.angular_velocity.n[i];
    imu_linear_acceleration[i] = dfr.filtered_imu.linear_acceleration.n[i];
  }
  
  imu_orientation_raw = Eigen::Quaterniond(dfr.filtered_imu.orientation_estimate.qw(), dfr.filtered_imu.orientation_estimate.qx(), dfr.filtered_imu.orientation_estimate.qy(), dfr.filtered_imu.orientation_estimate.qz());

  Eigen::Matrix3d rot = imu_orientation_raw.toRotationMatrix();
  rot = Eigen::AngleAxisd(-yaw, Eigen::Vector3d::UnitZ()) * rot;
  Eigen::Map <Eigen::Quaterniond> q(imu_orientation);
  q = Eigen::Quaterniond(rot);

  // foot force
  for (int side = LEFT; side < LR; side++) {
    foot_forces[side][2] = dfr.foot_sensors[side].fz;
    foot_forces[side][3] = dfr.foot_sensors[side].mx;
    foot_forces[side][4] = dfr.foot_sensors[side].my;

    if (fabs(foot_forces[side][2]) < 1)
      foot_forces[side][2] = 1;
  }

  // hand force
  for (int side = LEFT; side < LR; side++) {
    raw_hand_forces[side][0] = dfr.wrist_sensors[side].f.x();
    raw_hand_forces[side][1] = dfr.wrist_sensors[side].f.y();
    raw_hand_forces[side][2] = dfr.wrist_sensors[side].f.z();
    raw_hand_forces[side][3] = dfr.wrist_sensors[side].m.x();
    raw_hand_forces[side][4] = dfr.wrist_sensors[side].m.y();
    raw_hand_forces[side][5] = dfr.wrist_sensors[side].m.z();
  }

  // log bdi state estimator
  bdi_root_est[0] = dfr.pos_est.position.x();
  bdi_root_est[1] = dfr.pos_est.position.y();
  bdi_root_est[2] = dfr.pos_est.position.z();
  
  bdi_rootd_est[0] = dfr.pos_est.velocity.x();
  bdi_rootd_est[1] = dfr.pos_est.velocity.y();
  bdi_rootd_est[2] = dfr.pos_est.velocity.z();

  for (int side = 0; side < LR; side++) {
    bdi_foot_est[side][0] = dfr.foot_pos_est[side].x();
    bdi_foot_est[side][1] = dfr.foot_pos_est[side].y();
    bdi_foot_est[side][2] = dfr.foot_pos_est[side].z();
  }
}
*/ 
