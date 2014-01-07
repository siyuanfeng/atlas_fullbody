/*
 * =====================================================================================
 *
 *       Filename:  DummyFSPlanner.h
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  03/27/2013 08:32:51 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  YOUR NAME (), 
 *        Company:  
 *
 * =====================================================================================
 */


#ifndef __DUMMY_FS_PLANNER_H
#define __DUMMY_FS_PLANNER_H

#include "drc_common_defines.h"
#include "FootStep.h"
#include <vector>

void fsFromFile(const char *name, std::vector <SFootStep> &foot_steps);
void make_straight_fs_plan(const double start[6], const double end[6], double step_len, std::vector <SFootStep> &fs, int firstState = SSR, double step_width = 0.089);
void make_turn_fs_plan(const double start[6], const double end[6], std::vector <SFootStep> &fs, int firstState = DSc);

#endif
