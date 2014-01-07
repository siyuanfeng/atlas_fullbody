#include <ros/ros.h>
#include <signal.h>
#include <atlas_msgs/AtlasCommand.h>
#include <atlas_msgs/AtlasState.h>
#include <boost/thread.hpp>
#include <atlas_msgs/AtlasSimInterfaceCommand.h>

#include <cmu_walk/LipmWalkingCon.h>
#include <test_walk/cmu_ctrl_utils.h>

static void quit(int sig);

ros::Publisher pub_atlas_cmd;
ros::Subscriber sub_atlas_state;

boost::mutex state_lock;
atlas_msgs::AtlasState data_from_robot;
atlas_msgs::AtlasCommand data_to_robot;
CMUCtrlUtils utils;

/*
double rootq[4] = {0, 0, 0, 1};
double root[6] = {0, 0, 0.9545, 0, 0, 0};
double rootd[6] = {0};
double root_b_w[3] = {0};       // angular vel in body frame
double joints[N_JOINTS] = {0};
double jointsd[N_JOINTS] = {0}; 
double foot_forces[LR][6] = {{0}, {0}};
double imu_angular_velocity[3];
double imu_linear_acceleration[3];
double imu_orientation[4]; 
*/
 
void AtlasStateCallback(const atlas_msgs::AtlasState::ConstPtr &msg)
{
  boost::mutex::scoped_lock lock(state_lock);
  data_from_robot = *msg;
}  

int main(int argc, char **argv)
{
  signal( SIGINT, quit );
  ros::init(argc, argv, "test_walk", ros::init_options::NoSigintHandler);
  
  ros::NodeHandle nh;  
  ros::Time last_ros_time_;
  bool wait = true;
  while (wait) {
    last_ros_time_ = ros::Time::now();
    if (last_ros_time_.toSec() > 0) {
      wait = false;
    }
  }
  

  // ros pub / sub
  pub_atlas_cmd = nh.advertise<atlas_msgs::AtlasCommand>("/atlas/atlas_command", 1); 
  
  // ros topic subscriptions
  ros::SubscribeOptions jointStatesSo = ros::SubscribeOptions::create<atlas_msgs::AtlasState>("/atlas/atlas_state", 1, &AtlasStateCallback, ros::VoidPtr(), nh.getCallbackQueue());

  // Because TCP causes bursty communication with high jitter,
  // declare a preference on UDP connections for receiving
  // joint states, which we want to get at a high rate.
  // Note that we'll still accept TCP connections for this topic
  // (e.g., from rospy nodes, which don't support UDP);
  // we just prefer UDP.
  jointStatesSo.transport_hints = ros::TransportHints().unreliable();
  sub_atlas_state = nh.subscribe( jointStatesSo ); 

  // variables for walking
  int cState;
  PelvRobotState rs;            // contains all info about robot
  //HatRobotState rs;             // contains all info about robot
  KinematicFilter3 kcekf;       // state estimator
  LipmWalkingCon lwc;           // walking controller
  Command cmd;                  // contains outputs from controller
  bool inited = false;
  double last_rec_time = 0;

  // initialize controller
  load_KFParams(std::string("test_walk"), kcekf);
  lwc.allocCon(rs.getType());
  // load control params
  load_sf_params(
      std::string("test_walk"),
      std::string("/config/con_param/atlas_static_idCon.conf"),
      std::string("/config/con_param/atlas_static_ikCon.conf"),
      std::string("/config/con_param/atlas_wc.conf"),
      lwc); 
  
  //////////////////////////////////////////////////////////////
  // tell bdi i have control now!
  ros::Publisher pub_bdi_asi_cmd = nh.advertise<atlas_msgs::AtlasSimInterfaceCommand> 
    ( "/atlas/atlas_sim_interface_command", 1);
  atlas_msgs::AtlasSimInterfaceCommand bdi_cmd;
  bdi_cmd.header.stamp = ros::Time::now();
  bdi_cmd.behavior = atlas_msgs::AtlasSimInterfaceCommand::USER;
  pub_bdi_asi_cmd.publish(bdi_cmd);
  ////////////////////////////////////////////////////////////// 
  
  // main loop
  while (true) 
  {
    // tell ros to process all listening / publishing callbacks
    ros::spinOnce();
    
    // process data from simulator
    {
      boost::mutex::scoped_lock lock(state_lock);
      // haven't got the first real robot state from simulator
      if (data_from_robot.header.stamp.toSec() == 0)
        continue;
      
      // haven't got a new packet
      if (data_from_robot.header.stamp.toSec() == last_rec_time)
        continue;

      utils.UnpackDataFromRobot(data_from_robot);
      last_rec_time = data_from_robot.header.stamp.toSec();
    }
    
    // initialize state estimator to the first real robot state
    if (!inited) {
      for (int i = 0; i < N_JOINTS; i++)
        utils.f_mask[i] = CMUCtrlUtils::FF;
      utils.init_KF(kcekf, rs.getType(), 0.9545);
      utils.updateRobotState(DSc, rs);
      lwc.init(rs);
      inited = true;
    }
    // run state estimator and update robot state normally
    else {
      cState = lwc.getPlannedContactState(utils.time);
      utils.estimateState(cState, kcekf, utils.foot_forces[LEFT][ZZ], utils.foot_forces[RIGHT][ZZ]);
      utils.updateRobotState(cState, rs);
    }
    
    // run walking controller
    lwc.control(rs, cmd);
    utils.PackDataToRobot(cmd, rs.time, data_to_robot);

    // send commands to simulator
    pub_atlas_cmd.publish(data_to_robot);
  }
}




static void quit(int sig)
{
  
  ros::shutdown();
  ros::waitForShutdown(); 

  printf( "Goodbye\n" );
  exit(0);  
}

