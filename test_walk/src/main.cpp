#include <ros/ros.h>
#include <signal.h>
#include <atlas_msgs/AtlasCommand.h>
#include <atlas_msgs/AtlasState.h>
#include <boost/thread.hpp>
#include <atlas_msgs/AtlasSimInterfaceCommand.h>
#include <test_walk/cmu_ctrl_utils.h>
#include <test_walk/simple_text.h>
#include <test_walk/control_loop_sf_walk.h>
#include <test_walk/control_loop_ew_manip.h>

#define WALKING_CONTROLLER      0
#define MANIP_CONTROLLER        1
static ros::Publisher pub_atlas_cmd;
static ros::Subscriber sub_atlas_state;

static boost::mutex state_lock;
static atlas_msgs::AtlasState data_from_robot;
static atlas_msgs::AtlasCommand data_to_robot;

static test_walk::field_param user_input;
static test_walk::AtlasWalkParams step_input;
static test_walk::simple_text text_to_user;

static void quit(int sig);

std::string pkg_name("test_walk");

static void AtlasStateCallback(const atlas_msgs::AtlasState::ConstPtr &msg)
{
  boost::mutex::scoped_lock lock(state_lock);
  data_from_robot = *msg;
}  
 
int main(int argc, char **argv)
{
  signal( SIGINT, quit );
  ros::init(argc, argv, pkg_name, ros::init_options::NoSigintHandler);
  
  ros::NodeHandle nh;  
  ros::Time last_ros_time_;
  bool wait = true;
  while (wait) {
    last_ros_time_ = ros::Time::now();
    if (last_ros_time_.toSec() > 0) {
      wait = false;
    }
  }
  
  //////////////////////////////////////////////////////////////
  // ros pub / sub
  pub_atlas_cmd = nh.advertise<atlas_msgs::AtlasCommand>("/atlas/atlas_command", 10); 
  ros::SubscribeOptions jointStatesSo = ros::SubscribeOptions::create<atlas_msgs::AtlasState>("/atlas/atlas_state", 10, &AtlasStateCallback, ros::VoidPtr(), nh.getCallbackQueue());
  jointStatesSo.transport_hints = ros::TransportHints().reliable().tcpNoDelay(true);
  sub_atlas_state = nh.subscribe(jointStatesSo); 
  //////////////////////////////////////////////////////////////
  // tell the simulator that I have control now
  ros::Publisher pub_bdi_asi_cmd = nh.advertise<atlas_msgs::AtlasSimInterfaceCommand> 
    ( "/atlas/atlas_sim_interface_command", 1);
  atlas_msgs::AtlasSimInterfaceCommand bdi_cmd;
  bdi_cmd.header.stamp = ros::Time::now();
  bdi_cmd.behavior = atlas_msgs::AtlasSimInterfaceCommand::USER;
  pub_bdi_asi_cmd.publish(bdi_cmd);
  ////////////////////////////////////////////////////////////// 

  // init all controllers
  initialize_loop_sf_walk();
  initialize_loop_ew_manip();
  
  int which_controller = WALKING_CONTROLLER;
  int last_controller = MANIP_CONTROLLER;
  bool first_time = false;
  
  double last_rec_time = 0;

  // main loop
  while (1) 
  {
    // tell ros to process all listening / publishing callbacks
    ros::spinOnce();
    
    // check for new data from robot
    {
      boost::mutex::scoped_lock lock(state_lock);
      // haven't got the first real robot state from simulator
      if (data_from_robot.header.stamp.toSec() == 0)
        continue;
      
      // haven't got a new packet
      if (data_from_robot.header.stamp.toSec() == last_rec_time)
        continue;

      last_rec_time = data_from_robot.header.stamp.toSec();
    }

    // set first time flag
    if (which_controller != last_controller)
      first_time = true;
    else 
      first_time = false;
    last_controller = which_controller;

    // run control loop
    if (which_controller == WALKING_CONTROLLER) {
      control_loop_sf_walk(data_from_robot, state_lock, data_to_robot, user_input, step_input, first_time);
    }
    else if (which_controller == MANIP_CONTROLLER) {
      control_loop_ew_manip(data_from_robot, state_lock, data_to_robot, user_input, &text_to_user, first_time);
    }

    // send commands to simulator
    pub_atlas_cmd.publish(data_to_robot);
    
    ros::spinOnce();
  }    

  quit(SIGINT);
}


static void quit(int sig)
{
  ros::shutdown();
  ros::waitForShutdown(); 

  quit_sf_walk();
  quit_ew_manip();
  printf( "Goodbye\n" );
  exit(0);  
}
 
