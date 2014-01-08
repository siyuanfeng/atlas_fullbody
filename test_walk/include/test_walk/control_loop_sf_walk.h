#ifndef CONTROL_LOOP_SF_WALK_H
#define CONTROl_LOOP_SF_WALK_H

#include <atlas_msgs/AtlasCommand.h>
#include <atlas_msgs/AtlasState.h>
#include <test_walk/AtlasWalkParams.h>
#include <test_walk/field_param.h>
#include <boost/thread.hpp>

void control_loop_sf_walk(const atlas_msgs::AtlasState &data_from_robot,
    boost::mutex &data_from_robot_lock,
    atlas_msgs::AtlasCommand &data_to_robot,
    test_walk::field_param &params,
    test_walk::AtlasWalkParams &input_steps,
    bool firstTime);
void initialize_loop_sf_walk();
void quit_sf_walk();

#endif
