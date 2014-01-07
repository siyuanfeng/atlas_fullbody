#include "Command.h"
#include "Utils.hpp"
#include "RobotState.h"

double Command::BDI_joints[N_JOINTS] = 
{
  0, 0, 0., 
  0, 
  0, 0, -0.23, 0.52, -0.28, 0, 
  0, 0, -0.23, 0.52, -0.28, 0., 
  0.3, -1.3, 2, 0.5, 0.2, 0, 
  0.3, 1.3, 2, -0.5, 0.2, 0 
};

double Command::Eric_joints[N_JOINTS] = 
{
  0, 0, 0., 
  0, 
  0, 0, -0.23, 0.52, -0.28, 0, 
  0, 0, -0.23, 0.52, -0.28, 0., 
  -1.55, -1.15, 0.6, 1.9, 0.3, -1.1,
  -1.55, 1.15, 0.6, -1.9, 0.3, 1.1
};
 
Command::Command()
{
  dvec_set(jointsdd_d, 0, N_JOINTS);
  dvec_set(w_grf_d, 0, 12);
  dvec_set(b_grf_d, 0, 12);
  dvec_set(trq_ff, 0, N_JOINTS);
  dvec_set(comdd_d, 0, 3);
  dvec_set(pelvWd_d, 0, 3);
  dvec_set(torsoWd_d, 0, 3);
  dvec_set(footdd_d[LEFT], 0, 6);
  dvec_set(footdd_d[RIGHT], 0, 6);
  dvec_set(b_cop_d[LEFT], 0, 2);
  dvec_set(b_cop_d[RIGHT], 0, 2);
  dvec_set(cop_d, 0, 2);
 
  rootq_d = Eigen::Quaterniond::Identity();
  torsoq_d = Eigen::Quaterniond::Identity();
  dvec_set(root_d, 0, 3);
  dvec_set(rootd_d, 0, 3);
  dvec_set(joints_d, 0, N_JOINTS);
  dvec_set(jointsd_d, 0, N_JOINTS);
  dvec_set(com_d, 0, 3);
  dvec_set(comd_d, 0, 3);
  dvec_set(pelvW_d, 0, 3);
  dvec_set(torsoW_d, 0, 3);
  dvec_set(foot_d[LEFT], 0, 6);
  dvec_set(foot_d[RIGHT], 0, 6);
  dvec_set(footd_d[LEFT], 0, 6);
  dvec_set(footd_d[RIGHT], 0, 6);
}

/*
Command &Command::operator= (const Command &other)
{
  if (this == &other)
    return *this;
  
  dvec_copy(joints_d, other.joints_d, N_JOINTS); 
  dvec_copy(jointsd_d, other.jointsd_d, N_JOINTS); 
  dvec_copy(trq_ff, other.trq_ff, N_JOINTS); 
 
  dvec_copy(jointsdd_d, other.jointsdd_d, N_JOINTS);
  dvec_copy(w_grf_d, other.w_grf_d, 12);
  dvec_copy(trq_ff, other.trq_ff, N_JOINTS);
  dvec_copy(comdd_d, other.comdd_d, 3);
  dvec_copy(pelvWd_d, other.pelvWd_d, 3);
  dvec_copy(torsoWd_d, other.torsoWd_d, 3);
  dvec_copy(footdd_d[LEFT], other.footdd_d[LEFT], 6);
  dvec_copy(footdd_d[RIGHT], other.footdd_d[RIGHT], 6);
  dvec_copy(b_cop_d[LEFT], other.b_cop_d[LEFT], 2);
  dvec_copy(b_cop_d[RIGHT], other.b_cop_d[RIGHT], 2);
  dvec_copy(cop_d, other.cop_d, 2);
  
  dvec_copy(root_d, other.root_d, 3);
  dvec_copy(rootd_d, other.rootd_d, 3);
  dvec_copy(joints_d, other.joints_d, N_JOINTS);
  dvec_copy(jointsd_d, other.jointsd_d, N_JOINTS);
  dvec_copy(com_d, other.com_d, 3);
  dvec_copy(comd_d, other.comd_d, 3);
  dvec_copy(pelvW_d, other.pelvW_d, 3);
  dvec_copy(torsoW_d, other.torsoW_d, 3);
  dvec_copy(foot_d[LEFT], other.foot_d[LEFT], 6);
  dvec_copy(foot_d[RIGHT], other.foot_d[RIGHT], 6);
  dvec_copy(footd_d[LEFT], other.footd_d[LEFT], 6);
  dvec_copy(footd_d[RIGHT], other.footd_d[RIGHT], 6);

  return *this;
}
*/


void Command::addToLog(BatchLogger &logger)
{
  char buf[1000];
  // ID
	logger.add_datapoint("ID.comdd[X]","a",&(comdd_d[XX]));
	logger.add_datapoint("ID.comdd[Y]","a",&(comdd_d[YY]));
	logger.add_datapoint("ID.comdd[Z]","a",&(comdd_d[ZZ]));

	logger.add_datapoint("ID.rootdd[X]","a",&(rootdd_d[XX]));
	logger.add_datapoint("ID.rootdd[Y]","a",&(rootdd_d[YY]));
	logger.add_datapoint("ID.rootdd[Z]","a",&(rootdd_d[ZZ]));
  logger.add_datapoint("ID.pelvWd[X]","-",&(pelvWd_d[XX]));
  logger.add_datapoint("ID.pelvWd[Y]","-",&(pelvWd_d[YY]));
  logger.add_datapoint("ID.pelvWd[Z]","-",&(pelvWd_d[ZZ]));
	
  logger.add_datapoint("ID.torsoWd[X]","ra",&(torsoWd_d[XX]));
	logger.add_datapoint("ID.torsoWd[Y]","ra",&(torsoWd_d[YY]));
	logger.add_datapoint("ID.torsoWd[Z]","ra",&(torsoWd_d[ZZ]));

	logger.add_datapoint("ID.cop[X]","m",&(cop_d[XX]));
	logger.add_datapoint("ID.cop[Y]","m",&(cop_d[YY]));
  logger.add_datapoint("ID.b_cop[L][X]","m",&(b_cop_d[LEFT][XX]));
  logger.add_datapoint("ID.b_cop[L][Y]","m",&(b_cop_d[LEFT][YY]));
  logger.add_datapoint("ID.b_cop[R][X]","m",&(b_cop_d[RIGHT][XX]));
  logger.add_datapoint("ID.b_cop[R][Y]","m",&(b_cop_d[RIGHT][YY]));

	logger.add_datapoint("ID.ftdd[L][X]","a",&(footdd_d[LEFT][XX]));
	logger.add_datapoint("ID.ftdd[L][Y]","a",&(footdd_d[LEFT][YY]));
	logger.add_datapoint("ID.ftdd[L][Z]","a",&(footdd_d[LEFT][ZZ]));
	logger.add_datapoint("ID.ftdd[L][AX]","ra",&(footdd_d[LEFT][XX+3]));
	logger.add_datapoint("ID.ftdd[L][AY]","ra",&(footdd_d[LEFT][YY+3]));
	logger.add_datapoint("ID.ftdd[L][AZ]","ra",&(footdd_d[LEFT][ZZ+3]));
	
  logger.add_datapoint("ID.ftdd[R][X]","a",&(footdd_d[RIGHT][XX]));
	logger.add_datapoint("ID.ftdd[R][Y]","a",&(footdd_d[RIGHT][YY]));
	logger.add_datapoint("ID.ftdd[R][Z]","a",&(footdd_d[RIGHT][ZZ]));
	logger.add_datapoint("ID.ftdd[R][AX]","ra",&(footdd_d[RIGHT][XX+3]));
	logger.add_datapoint("ID.ftdd[R][AY]","ra",&(footdd_d[RIGHT][YY+3]));
	logger.add_datapoint("ID.ftdd[R][AZ]","ra",&(footdd_d[RIGHT][ZZ+3]));
  
  logger.add_datapoint("ID.b_F[L][X]", "N", &(b_grf_d[XX]));
  logger.add_datapoint("ID.b_F[L][Y]", "N", &(b_grf_d[YY]));
  logger.add_datapoint("ID.b_F[L][Z]", "N", &(b_grf_d[ZZ]));
  logger.add_datapoint("ID.b_M[L][X]", "Nm", &(b_grf_d[XX+3]));
  logger.add_datapoint("ID.b_M[L][Y]", "Nm", &(b_grf_d[YY+3]));
  logger.add_datapoint("ID.b_M[L][Z]", "Nm", &(b_grf_d[ZZ+3]));
  logger.add_datapoint("ID.b_F[R][X]", "N", &(b_grf_d[XX+6]));
  logger.add_datapoint("ID.b_F[R][Y]", "N", &(b_grf_d[YY+6]));
  logger.add_datapoint("ID.b_F[R][Z]", "N", &(b_grf_d[ZZ+6]));
  logger.add_datapoint("ID.b_M[R][X]", "Nm", &(b_grf_d[XX+9]));
  logger.add_datapoint("ID.b_M[R][Y]", "Nm", &(b_grf_d[YY+9]));
  logger.add_datapoint("ID.b_M[R][Z]", "Nm", &(b_grf_d[ZZ+9]));
  
  for (int i = 0; i < N_JOINTS; i++) {
    sprintf(buf,"ID.%s_acc", RobotState::joint_names[i].c_str());
    logger.add_datapoint(buf, "r/s/s", &(jointsdd_d[i]));
  }
  
  for (int i = 0; i < N_JOINTS; i++) {
    sprintf(buf,"ID.%s_trq", RobotState::joint_names[i].c_str());
    logger.add_datapoint(buf, "Nm", &(trq_ff[i]));
  }

  logger.add_datapoint("ID.w_F[L][X]", "N", &(w_grf_d[XX]));
  logger.add_datapoint("ID.w_F[L][Y]", "N", &(w_grf_d[YY]));
  logger.add_datapoint("ID.w_F[L][Z]", "N", &(w_grf_d[ZZ]));
  logger.add_datapoint("ID.w_M[L][X]", "Nm", &(w_grf_d[XX+3]));
  logger.add_datapoint("ID.w_M[L][Y]", "Nm", &(w_grf_d[YY+3]));
  logger.add_datapoint("ID.w_M[L][Z]", "Nm", &(w_grf_d[ZZ+3]));
  logger.add_datapoint("ID.w_F[R][X]", "N", &(w_grf_d[XX+6]));
  logger.add_datapoint("ID.w_F[R][Y]", "N", &(w_grf_d[YY+6]));
  logger.add_datapoint("ID.w_F[R][Z]", "N", &(w_grf_d[ZZ+6]));
  logger.add_datapoint("ID.w_M[R][X]", "Nm", &(w_grf_d[XX+9]));
  logger.add_datapoint("ID.w_M[R][Y]", "Nm", &(w_grf_d[YY+9]));
  logger.add_datapoint("ID.w_M[R][Z]", "Nm", &(w_grf_d[ZZ+9]));
  
  // IK
  logger.add_datapoint("IK.root[X]","m",&(root_d[XX]));
	logger.add_datapoint("IK.root[Y]","m",&(root_d[YY]));
	logger.add_datapoint("IK.root[Z]","m",&(root_d[ZZ]));
  logger.add_quat("IK.pelvQ",&(rootq_d));
  logger.add_quat("IK.torsoQ",&(torsoq_d));
   
  logger.add_datapoint("IK.com[X]","m",&(com_d[XX]));
	logger.add_datapoint("IK.com[Y]","m",&(com_d[YY]));
	logger.add_datapoint("IK.com[Z]","m",&(com_d[ZZ]));

	logger.add_datapoint("IK.comd[X]","v",&(comd_d[XX]));
	logger.add_datapoint("IK.comd[Y]","v",&(comd_d[YY]));
	logger.add_datapoint("IK.comd[Z]","v",&(comd_d[ZZ]));

	logger.add_datapoint("IK.foot[L][X]","m",&(foot_d[LEFT][XX]));
	logger.add_datapoint("IK.foot[L][Y]","m",&(foot_d[LEFT][YY]));
	logger.add_datapoint("IK.foot[L][Z]","m",&(foot_d[LEFT][ZZ]));
	logger.add_datapoint("IK.foot[R][X]","m",&(foot_d[RIGHT][XX]));
	logger.add_datapoint("IK.foot[R][Y]","m",&(foot_d[RIGHT][YY]));
	logger.add_datapoint("IK.foot[R][Z]","m",&(foot_d[RIGHT][ZZ]));
  logger.add_quat("IK.foot[L].Q",&(footq_d[LEFT]));
  logger.add_quat("IK.foot[R].Q",&(footq_d[RIGHT]));

  logger.add_datapoint("IK.footd[L][X]","m",&(footd_d[LEFT][XX]));
	logger.add_datapoint("IK.footd[L][Y]","m",&(footd_d[LEFT][YY]));
	logger.add_datapoint("IK.footd[L][Z]","m",&(footd_d[LEFT][ZZ]));
	logger.add_datapoint("IK.footd[R][X]","m",&(footd_d[RIGHT][XX]));
	logger.add_datapoint("IK.footd[R][Y]","m",&(footd_d[RIGHT][YY]));
	logger.add_datapoint("IK.footd[R][Z]","m",&(footd_d[RIGHT][ZZ])); 

	logger.add_datapoint("IK.pelvW[X]","r/s",&(pelvW_d[XX]));
	logger.add_datapoint("IK.pelvW[Y]","r/s",&(pelvW_d[YY]));
	logger.add_datapoint("IK.pelvW[Z]","r/s",&(pelvW_d[ZZ]));

	logger.add_datapoint("IK.torsoW[X]","r/s",&(torsoW_d[XX]));
	logger.add_datapoint("IK.torsoW[Y]","r/s",&(torsoW_d[YY]));
	logger.add_datapoint("IK.torsoW[Z]","r/s",&(torsoW_d[ZZ]));
	
  for (int i = 0; i < N_JOINTS; i++) {
    sprintf(buf,"IK.%s_d", RobotState::joint_names[i].c_str());
    logger.add_datapoint(buf, "rad", &(joints_d[i]));
  }
  
  for (int i = 0; i < N_JOINTS; i++) {
    sprintf(buf,"IK.%sd_d", RobotState::joint_names[i].c_str());
    logger.add_datapoint(buf, "rad/s", &(jointsd_d[i]));
  }
}
