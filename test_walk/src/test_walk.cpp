#include <ros/ros.h>
#include <signal.h>
#include <atlas_msgs/AtlasCommand.h>
#include <atlas_msgs/AtlasState.h>
#include <boost/thread.hpp>

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
  
  ros::Time last_ros_time_;
  bool wait = true;
  while (wait) {
    last_ros_time_ = ros::Time::now();
    if (last_ros_time_.toSec() > 0) {
      wait = false;
    }
  }
  

  // ros pub / sub
  ros::NodeHandle nh;  

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
}




static void quit(int sig)
{
  
}

