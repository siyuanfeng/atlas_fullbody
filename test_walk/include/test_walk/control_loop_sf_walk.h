#ifndef CONTROL_LOOP_SF_WALK_H
#define CONTROl_LOOP_SF_WALK_H

#include <atlas_msgs/AtlasCommand.h>
#include <atlas_msgs/AtlasState.h>
#include <boost/thread.hpp>

#include <atlas_ros_msgs/AtlasWalkParams.h>
#include <atlas_ros_msgs/field_param.h>
#include <atlas_ros_msgs/sf_state_est.h>

void control_loop_sf_walk(const atlas_msgs::AtlasState &data_from_robot,
    boost::mutex &data_from_robot_lock,
    atlas_msgs::AtlasCommand &data_to_robot,
    atlas_ros_msgs::field_param &params,
    atlas_ros_msgs::AtlasWalkParams &input_steps,
    atlas_ros_msgs::sf_state_est &est_out,
    bool firstTime);
void initialize_loop_sf_walk();
void quit_sf_walk();

#endif
