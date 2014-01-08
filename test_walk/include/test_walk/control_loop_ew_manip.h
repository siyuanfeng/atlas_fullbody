#ifndef CONTROL_LOOP_EW_MANIP_H
#define CONTROl_LOOP_EW_MANIP_H

#include <atlas_msgs/AtlasCommand.h>
#include <atlas_msgs/AtlasState.h>
#include <test_walk/field_param.h>
#include <test_walk/simple_text.h>

void control_loop_ew_manip(const atlas_msgs::AtlasState &data_from_robot,
    boost::mutex &data_from_robot_lock,
    atlas_msgs::AtlasCommand &data_to_robot,
    test_walk::field_param &params,
    test_walk::simple_text *to_ocu,
    bool firstTime);
void initialize_loop_ew_manip();
void quit_ew_manip();

#endif 
