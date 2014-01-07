#ifndef __COMMAND_H
#define __COMMAND_H

#include "drc_common_defines.h"
#include <eigen3/Eigen/Geometry>
#include "Logger.h"

class Command {
  public:
    static double BDI_joints[N_JOINTS];
    static double Eric_joints[N_JOINTS];
   
    // result of ID
    double jointsdd_d[N_JOINTS];
    double w_grf_d[12];
    double b_grf_d[12];
    double trq_ff[N_JOINTS];

    double comdd_d[3];
    double rootdd_d[3];
    double pelvWd_d[3];
    double torsoWd_d[3];
    double footdd_d[LR][6];
    double b_cop_d[LR][2];
    double cop_d[2];

    // result of IK
    double root_d[3];
    Eigen::Quaterniond rootq_d;
    Eigen::Quaterniond torsoq_d;
    double rootd_d[3];
    double joints_d[N_JOINTS];
    double jointsd_d[N_JOINTS];
   
    Eigen::Quaterniond footq_d[LR];
    double com_d[3];
    double comd_d[3];
    double pelvW_d[3];
    double torsoW_d[3];
    double foot_d[LR][6];
    double footd_d[LR][6];

    Command();
    //Command &operator= (const Command &other); 
    void addToLog(BatchLogger &logger);
};
  
#endif
