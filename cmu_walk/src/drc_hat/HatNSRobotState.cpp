#include "RobotState.h"
#include "drc_hat_ns.h"
#include "drc_hat_ns_defines.h"
#include "SDModel_common.h"
#include "Utils.hpp"
#include "Eigen_utils.hpp"

HatNSRobotState::HatNSRobotState()
{
  _type = RobotState::TYPE_HAT_NS;
  _sdFastState = new double[HATNS_N_SDFAST_STATE];
  model = new drc_hat_ns();

#ifdef ATLAS_ONLINE
  double btj[3] = {0};
  model->sdgetbtj(HATNS_S_JOINT_BACK_UBX, btj);
  //btj[1] = -0.0092;
  fprintf(stderr, "btj %g %g %g\n", btj[0], btj[1], btj[2]);
  model->sdbtj(HATNS_S_JOINT_BACK_UBX, btj);
#endif

  model->sdinit();

  for (int i = 0; i < LR; i++) {
    J[i] = Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor>::Zero(HATNS_N_SDFAST_U,6);
    Jd[i] = Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor>::Zero(HATNS_N_SDFAST_U,6);
    Jtoe[i] = Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor>::Zero(HATNS_N_SDFAST_U,6);
    Jtoed[i] = Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor>::Zero(HATNS_N_SDFAST_U,6);
    Jheel[i] = Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor>::Zero(HATNS_N_SDFAST_U,6);
    Jheeld[i] = Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor>::Zero(HATNS_N_SDFAST_U,6);
    Jmid[i] = Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor>::Zero(HATNS_N_SDFAST_U,6);
    Jmidd[i] = Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor>::Zero(HATNS_N_SDFAST_U,6);
  }

  Jc = Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor>::Zero(HATNS_N_SDFAST_U,3); 
  Jcd = Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor>::Zero(HATNS_N_SDFAST_U,3); 
  
  Jtorso = Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor>::Zero(HATNS_N_SDFAST_U,6); 
  Jtorsod = Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor>::Zero(HATNS_N_SDFAST_U,6); 
	Jpelvis = Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor>::Zero(HATNS_N_SDFAST_U,6);
	Jpelvisd = Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor>::Zero(HATNS_N_SDFAST_U,6);

  M = Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor>::Zero(HATNS_N_SDFAST_U,HATNS_N_SDFAST_U); 
  nonLin = Eigen::Matrix<double,Eigen::Dynamic,1>::Zero(HATNS_N_SDFAST_U);  
}

HatNSRobotState::~HatNSRobotState()
{
  delete []_sdFastState;
  delete model;
}

void HatNSRobotState::computeSDFvars()
{
  makeSDFastState(root, rootq, rootd, root_b_w, joints, jointsd, _sdFastState);
  set_state(model, _sdFastState);
  
	double temp[6];
  double zeros[3] = {0, 0, 0};
  const int joint[2] = {HATNS_S_JOINT_L_LEG_LAX, HATNS_S_JOINT_R_LEG_LAX};
  const int body[2] = {HATNS_S_BODY_L_FOOT, HATNS_S_BODY_R_FOOT};
	//momentem and KE
	model->sdmom(mom, angMom, &KE);
  double dircos[3][3];
  double qq[4];
  double com_to_ankle[3];

  dvec_copy(temp, root2tfRoot, 3);
  model->sdpos(HATNS_S_BODY_PELVIS, temp, tfRoot);

  for (int side = 0; side < 2; side++) {
    model->sdgetbtj(joint[side], com_to_ankle);

    //foot positions and jacobians
    dvec_add(com_to_ankle, Foot::ankle_to_sensor_offset, temp, 3);
    body_position(model, NULL, body[side], temp, feet[side].w_sensor_pos, J[side].data());
    model->sdvel(body[side], temp, feet[side].w_sensor_vel);

    //mid of foot pos and vel
    dvec_add(com_to_ankle, feet[side].ankle_to_center_offset, temp, 3);
    body_position(model, NULL, body[side], temp, feet[side].w_mid_pos, Jmid[side].data());
    model->sdvel(body[side], temp, feet[side].w_mid_vel);
    
    //toe pos and vel
    dvec_add(com_to_ankle, feet[side].ankle_to_toe_offset, temp, 3);
    body_position(model, NULL, body[side], temp, feet[side].w_toe_pos, Jtoe[side].data());
    model->sdvel(body[side], temp, feet[side].w_toe_vel);

    //heel pos and vel
    dvec_add(com_to_ankle, feet[side].ankle_to_heel_offset, temp, 3);
    body_position(model, NULL, body[side], temp, feet[side].w_heel_pos, Jheel[side].data());
    model->sdvel(body[side], temp, feet[side].w_heel_vel);

    //foot quaternion
    model->sdorient(body[side], dircos);
    model->sddc2quat(dircos, qq, qq+1, qq+2, qq+3);
    feet[side].w_q = Eigen::Quaterniond(qq[3], qq[0], qq[1], qq[2]).normalized();
    //foot angular velocities
    model->sdangvel(body[side], feet[side].b_w);
    model->sdtrans(body[side], feet[side].b_w, -1, &(feet[side].w_sensor_vel[3]));

    dvec_copy(feet[side].w_mid_vel+3, feet[side].w_sensor_vel+3, 3);
    dvec_copy(feet[side].w_toe_vel+3, feet[side].w_sensor_vel+3, 3);
    dvec_copy(feet[side].w_heel_vel+3, feet[side].w_sensor_vel+3, 3);

    // foot tip
    for (int i = 0; i < 4; i++) {
      dvec_add(com_to_ankle, Foot::ankle_to_tip_offset[i], temp, 3);
      
      model->sdpos(body[side], temp, feet[side].vertices[i]);
    }
  }

  model->sdorient(HATNS_S_BODY_PELVIS, dircos);
  model->sddc2ang(dircos, root+3, root+4, root+5);

  /*
  // fill out various body part locations
  double offset[3];
  for (int i = 1; i < N_BODIES; i++) {
    model->sdgetbtj(i, offset);
    model->sdpos(i, offset, &(locations[i-1][0]));
  }
  */
  
  //mass, com position, and total moment of inertia
  model->sdsys(&m, com, (double (*)[3])I.data());

  //comd from momentum
  for(int i = 0; i < 3; i++)
    comd[i] = mom[i]/m;

  // pelvis angular velocity in world frame
  model->sdangvel(HATNS_S_BODY_PELVIS, temp);
  model->sdtrans(HATNS_S_BODY_PELVIS, temp, -1, rootd+3);

  // utorso orientation
  body_position(model, NULL, HATNS_S_BODY_HAT, zeros, temp, Jtorso.data());
  model->sdorient(HATNS_S_BODY_HAT, dircos);
  model->sddc2quat(dircos, qq, qq+1, qq+2, qq+3);
  utorsoq = Eigen::Quaterniond(qq[3], qq[0], qq[1], qq[2]).normalized();
  model->sdangvel(HATNS_S_BODY_HAT, temp);
  model->sdtrans(HATNS_S_BODY_HAT, temp, -1, utorsow);
  
  // pelvis orientation
	body_position(model, NULL, HATNS_S_BODY_PELVIS, zeros, temp, Jpelvis.data());
	model->sdorient(HATNS_S_BODY_PELVIS, dircos);
	model->sddc2quat(dircos, qq, qq+1, qq+2, qq+3);
	pelvisq = Eigen::Quaterniond(qq[3], qq[0], qq[1], qq[2]).normalized();
	model->sdangvel(HATNS_S_BODY_PELVIS, temp);
	model->sdtrans(HATNS_S_BODY_PELVIS, temp, -1, pelvisw); 
  
  //mass matrix
	model->sdmassmat(M.data());

	//com jacobian
	com_jacobian(model, Jc.data());

  //com inertia
	model->sdgetiner(HATNS_S_BODY_PELVIS, (double (*)[3])Itorso.data());

  // non lin part
  model->sdfrcmat(nonLin.data());

  // compute Jd and Jcd by integrating one step to get next J
  ///////////////////////////////////////////////////////////////////////////////////
  Eigen::Matrix<double,HATNS_N_SDFAST_U,6,Eigen::RowMajor> J2[LR];
  Eigen::Matrix<double,HATNS_N_SDFAST_U,3,Eigen::RowMajor> Jc2;
  Eigen::Matrix<double,HATNS_N_SDFAST_U,6,Eigen::RowMajor> Jtorso2;
  Eigen::Matrix<double,HATNS_N_SDFAST_U,6,Eigen::RowMajor> Jpelvis2;

  double qdot[HATNS_N_SDFAST_Q];
  model->sdu2qdot(&_sdFastState[HATNS_N_SDFAST_Q], qdot);
  // set new state
  for (int i = 0; i < HATNS_N_SDFAST_Q; i++)
    _sdFastState[i] += qdot[i] * timeStep; 
  set_state(model, _sdFastState);

  // foot positions and jacobians
  for (int side = 0; side < 2; side++) {
    model->sdgetbtj(joint[side], com_to_ankle);
    // ref point
    dvec_add(com_to_ankle, Foot::ankle_to_sensor_offset, temp, 3);
    body_position(model, NULL, body[side], temp, NULL, J2[side].data());
    Jd[side] = (J2[side] - J[side]) / timeStep;

    //mid of foot
    dvec_add(com_to_ankle, feet[side].ankle_to_center_offset, temp, 3);
    body_position(model, NULL, body[side], temp, NULL, J2[side].data());
    Jmidd[side] = (J2[side] - Jmid[side]) / timeStep;
    
    //toe
    dvec_add(com_to_ankle, feet[side].ankle_to_toe_offset, temp, 3);
    body_position(model, NULL, body[side], temp, NULL, J2[side].data());
    Jtoed[side] = (J2[side] - Jtoe[side]) / timeStep;

    //heel
    dvec_add(com_to_ankle, feet[side].ankle_to_heel_offset, temp, 3);
    body_position(model, NULL, body[side], temp, NULL, J2[side].data());
    Jheeld[side] = (J2[side] - Jheel[side]) / timeStep;
  }
  // com
	com_jacobian(model, Jc2.data());
  Jcd = (Jc2 - Jc) / timeStep;

  // utorso
  body_position(model, NULL, HATNS_S_BODY_HAT, zeros, temp, Jtorso2.data());
  Jtorsod = (Jtorso2 - Jtorso) / timeStep;
  
  //pelvis
	body_position(model, NULL, HATNS_S_BODY_PELVIS, zeros, temp, Jpelvis2.data());
	Jpelvisd = (Jpelvis2 - Jpelvis) / timeStep; 
  
  // restore
  for (int i = 0; i < HATNS_N_SDFAST_Q; i++)
    _sdFastState[i] -= qdot[i] * timeStep; 
  set_state(model, _sdFastState); 
  ///////////////////////////////////////////////////////////////////////////////////
  
  model->sdprinterr(stderr);
  //sdclearerr(); 
}

void HatNSRobotState::makeSDFastState(const double pos[3], const Eigen::Quaterniond &q, 
    const double vel[3], const double w[3], 
    const double j[N_JOINTS], const double jd[N_JOINTS], 
    double *sdfast_state)
{
  for (int i = 0; i < HATNS_N_SDFAST_STATE; i++)
    sdfast_state[i] = 0;
  
  // copy joint pos and vel
  pelv2hatNS(j, sdfast_state+6);
  pelv2hatNS(jd, sdfast_state+HATNS_N_SDFAST_Q+6);
  
  // get position and vel 
  memcpy(sdfast_state, pos, sizeof(double)*XYZ);
  sdfast_state[HATNS_S_Q0] = q.x();
  sdfast_state[HATNS_S_Q1] = q.y();
  sdfast_state[HATNS_S_Q2] = q.z();
  sdfast_state[HATNS_S_Q3] = q.w();

  memcpy(sdfast_state+HATNS_N_SDFAST_Q, vel, sizeof(double)*XYZ);
  memcpy(sdfast_state+HATNS_N_SDFAST_Q+3, w, sizeof(double)*XYZ);  
}

int HatNSRobotState::pelvIdx2hatNSIdx(int i)
{
  switch (i) {
    case A_L_LEG_UHZ:
      return HATNS_A_L_LEG_UHZ;
    case A_L_LEG_MHX:
      return HATNS_A_L_LEG_MHX;
    case A_L_LEG_LHY:
      return HATNS_A_L_LEG_LHY;
    case A_L_LEG_KNY:
      return HATNS_A_L_LEG_KNY;
    case A_L_LEG_UAY:
      return HATNS_A_L_LEG_UAY;
    case A_L_LEG_LAX:
      return HATNS_A_L_LEG_LAX;
    case A_R_LEG_UHZ:
      return HATNS_A_R_LEG_UHZ;
    case A_R_LEG_MHX:
      return HATNS_A_R_LEG_MHX;
    case A_R_LEG_LHY:
      return HATNS_A_R_LEG_LHY;
    case A_R_LEG_KNY:
      return HATNS_A_R_LEG_KNY;
    case A_R_LEG_UAY:
      return HATNS_A_R_LEG_UAY;
    case A_R_LEG_LAX:
      return HATNS_A_R_LEG_LAX;
    default:
      return -1;
  }
}

int HatNSRobotState::hatNSIdx2pelvIdx(int i)
{
  switch (i) {
    case HATNS_A_L_LEG_UHZ:
      return A_L_LEG_UHZ;
    case HATNS_A_L_LEG_MHX:
      return A_L_LEG_MHX;
    case HATNS_A_L_LEG_LHY:
      return A_L_LEG_LHY;
    case HATNS_A_L_LEG_KNY:
      return A_L_LEG_KNY;
    case HATNS_A_L_LEG_UAY:
      return A_L_LEG_UAY;
    case HATNS_A_L_LEG_LAX:
      return A_L_LEG_LAX;
    case HATNS_A_R_LEG_UHZ:
      return A_R_LEG_UHZ;
    case HATNS_A_R_LEG_MHX:
      return A_R_LEG_MHX;
    case HATNS_A_R_LEG_LHY:
      return A_R_LEG_LHY;
    case HATNS_A_R_LEG_KNY:
      return A_R_LEG_KNY;
    case HATNS_A_R_LEG_UAY:
      return A_R_LEG_UAY;
    case HATNS_A_R_LEG_LAX:
      return A_R_LEG_LAX;
    default:
      return -1;
  }
}

void HatNSRobotState::pelv2hatNS(const double pelv[N_JOINTS], double hat[HATNS_N_JOINTS])
{
  hat[HATNS_A_L_LEG_UHZ] = pelv[A_L_LEG_UHZ];
  hat[HATNS_A_L_LEG_MHX] = pelv[A_L_LEG_MHX];
  hat[HATNS_A_L_LEG_LHY] = pelv[A_L_LEG_LHY];
  hat[HATNS_A_L_LEG_KNY] = pelv[A_L_LEG_KNY];
  hat[HATNS_A_L_LEG_UAY] = pelv[A_L_LEG_UAY];
  hat[HATNS_A_L_LEG_LAX] = pelv[A_L_LEG_LAX];
  hat[HATNS_A_R_LEG_UHZ] = pelv[A_R_LEG_UHZ];
  hat[HATNS_A_R_LEG_MHX] = pelv[A_R_LEG_MHX];
  hat[HATNS_A_R_LEG_LHY] = pelv[A_R_LEG_LHY];
  hat[HATNS_A_R_LEG_KNY] = pelv[A_R_LEG_KNY];
  hat[HATNS_A_R_LEG_UAY] = pelv[A_R_LEG_UAY];
  hat[HATNS_A_R_LEG_LAX] = pelv[A_R_LEG_LAX];
}

void HatNSRobotState::hatNS2pelv(const double hat[HATNS_N_JOINTS], double pelv[N_JOINTS])
{
  pelv[A_L_LEG_UHZ] = hat[HATNS_A_L_LEG_UHZ];
  pelv[A_L_LEG_MHX] = hat[HATNS_A_L_LEG_MHX];
  pelv[A_L_LEG_LHY] = hat[HATNS_A_L_LEG_LHY];
  pelv[A_L_LEG_KNY] = hat[HATNS_A_L_LEG_KNY];
  pelv[A_L_LEG_UAY] = hat[HATNS_A_L_LEG_UAY];
  pelv[A_L_LEG_LAX] = hat[HATNS_A_L_LEG_LAX];
  pelv[A_R_LEG_UHZ] = hat[HATNS_A_R_LEG_UHZ];
  pelv[A_R_LEG_MHX] = hat[HATNS_A_R_LEG_MHX];
  pelv[A_R_LEG_LHY] = hat[HATNS_A_R_LEG_LHY];
  pelv[A_R_LEG_KNY] = hat[HATNS_A_R_LEG_KNY];
  pelv[A_R_LEG_UAY] = hat[HATNS_A_R_LEG_UAY];
  pelv[A_R_LEG_LAX] = hat[HATNS_A_R_LEG_LAX]; 
}
 
void HatNSRobotState::addToLog(BatchLogger &logger)
{
	logger.add_datapoint("RS.time","s",&time);
  logger.add_datapoint("RS.cState", "-", &contactState);

	logger.add_datapoint("RS.com[X]","m",&(com[XX]));
	logger.add_datapoint("RS.com[Y]","m",&(com[YY]));
	logger.add_datapoint("RS.com[Z]","m",&(com[ZZ]));
  logger.add_datapoint("RS.comd[X]","m/s",&(comd[XX]));
	logger.add_datapoint("RS.comd[Y]","m/s",&(comd[YY]));
	logger.add_datapoint("RS.comd[Z]","m/s",&(comd[ZZ]));
	
  logger.add_quat("RS.pelvQ",&(pelvisq));
	logger.add_quat("RS.torsoQ",&(utorsoq));
   
	logger.add_datapoint("RS.cop[X]","m",&(cop[XX]));
	logger.add_datapoint("RS.cop[Y]","m",&(cop[YY]));
  logger.add_datapoint("RS.foot[L].b_cop[X]","m",&(feet[LEFT].b_cop[XX]));
  logger.add_datapoint("RS.foot[L].b_cop[Y]","m",&(feet[LEFT].b_cop[YY]));
  logger.add_datapoint("RS.foot[R].b_cop[X]","m",&(feet[RIGHT].b_cop[XX]));
  logger.add_datapoint("RS.foot[R].b_cop[Y]","m",&(feet[RIGHT].b_cop[YY]));

  logger.add_quat("RS.footQ[L]",&(feet[LEFT].w_q));
	logger.add_quat("RS.footQ[R]",&(feet[RIGHT].w_q));

	logger.add_datapoint("RS.foot[L][X]","m",&(feet[LEFT].w_sensor_pos[XX]));
	logger.add_datapoint("RS.foot[L][Y]","m",&(feet[LEFT].w_sensor_pos[YY]));
	logger.add_datapoint("RS.foot[L][Z]","m",&(feet[LEFT].w_sensor_pos[ZZ]));
	logger.add_datapoint("RS.foot[R][X]","m",&(feet[RIGHT].w_sensor_pos[XX]));
	logger.add_datapoint("RS.foot[R][Y]","m",&(feet[RIGHT].w_sensor_pos[YY]));
	logger.add_datapoint("RS.foot[R][Z]","m",&(feet[RIGHT].w_sensor_pos[ZZ]));
  logger.add_datapoint("RS.footd[L][X]","m/s",&(feet[LEFT].w_sensor_vel[XX]));
	logger.add_datapoint("RS.footd[L][Y]","m/s",&(feet[LEFT].w_sensor_vel[YY]));
	logger.add_datapoint("RS.footd[L][Z]","m/s",&(feet[LEFT].w_sensor_vel[ZZ]));
	logger.add_datapoint("RS.footd[R][X]","m/s",&(feet[RIGHT].w_sensor_vel[XX]));
	logger.add_datapoint("RS.footd[R][Y]","m/s",&(feet[RIGHT].w_sensor_vel[YY]));
	logger.add_datapoint("RS.footd[R][Z]","m/s",&(feet[RIGHT].w_sensor_vel[ZZ])); 
	
  logger.add_datapoint("RS.footW[L][X]","m/s",&(feet[LEFT].w_sensor_vel[XX+3]));
	logger.add_datapoint("RS.footW[L][Y]","m/s",&(feet[LEFT].w_sensor_vel[YY+3]));
	logger.add_datapoint("RS.footW[L][Z]","m/s",&(feet[LEFT].w_sensor_vel[ZZ+3])); 
  logger.add_datapoint("RS.footW[R][X]","m/s",&(feet[RIGHT].w_sensor_vel[XX+3]));
	logger.add_datapoint("RS.footW[R][Y]","m/s",&(feet[RIGHT].w_sensor_vel[YY+3]));
	logger.add_datapoint("RS.footW[R][Z]","m/s",&(feet[RIGHT].w_sensor_vel[ZZ+3]));  
  
  logger.add_datapoint("RS.pelvW[X]","r/s",&(pelvisw[XX]));
	logger.add_datapoint("RS.pelvW[Y]","r/s",&(pelvisw[YY]));
	logger.add_datapoint("RS.pelvW[Z]","r/s",&(pelvisw[ZZ]));

	logger.add_datapoint("RS.torsoW[X]","r/s",&(utorsow[XX]));
	logger.add_datapoint("RS.torsoW[Y]","r/s",&(utorsow[YY]));
	logger.add_datapoint("RS.torsoW[Z]","r/s",&(utorsow[ZZ]));

	//logger.add_datapoint("RS.F[L][X]","N",&(feet[LEFT].w_F[XX]));
	//logger.add_datapoint("RS.F[L][Y]","N",&(feet[LEFT].w_F[YY]));
	logger.add_datapoint("RS.F[L][Z]","N",&(feet[LEFT].w_F[ZZ]));
	//logger.add_datapoint("RS.F[R][X]","N",&(feet[RIGHT].w_F[XX]));
	//logger.add_datapoint("RS.F[R][Y]","N",&(feet[RIGHT].w_F[YY]));
	logger.add_datapoint("RS.F[R][Z]","N",&(feet[RIGHT].w_F[ZZ]));

	logger.add_datapoint("RS.M[L][X]","Nm",&(feet[LEFT].w_F[XX+3]));
	logger.add_datapoint("RS.M[L][Y]","Nm",&(feet[LEFT].w_F[YY+3]));
	//logger.add_datapoint("RS.M[L][Z]","Nm",&(feet[LEFT].w_F[ZZ+3]));
	logger.add_datapoint("RS.M[R][X]","Nm",&(feet[RIGHT].w_F[XX+3]));
	logger.add_datapoint("RS.M[R][Y]","Nm",&(feet[RIGHT].w_F[YY+3]));
	//logger.add_datapoint("RS.M[R][Z]","Nm",&(feet[RIGHT].w_F[ZZ+3]));

	logger.add_datapoint("RS.root[X]","m",&(root[XX]));
	logger.add_datapoint("RS.root[Y]","m",&(root[YY]));
	logger.add_datapoint("RS.root[Z]","m",&(root[ZZ]));
	logger.add_datapoint("RS.rootd[X]","m",&(rootd[XX]));
	logger.add_datapoint("RS.rootd[Y]","m",&(rootd[YY]));
	logger.add_datapoint("RS.rootd[Z]","m",&(rootd[ZZ]));
	
  char buf[1000];
  for (int i = 0; i < N_JOINTS; i++) {
    sprintf(buf,"RS.%s", RobotState::joint_names[i].c_str());
    logger.add_datapoint(buf, "rad", &(joints[i]));
  }
  for (int i = 0; i < N_JOINTS; i++) {
    sprintf(buf,"RS.%sd", RobotState::joint_names[i].c_str());
    logger.add_datapoint(buf, "rad/s", &(jointsd[i]));
  }
  for (int i = 0; i < N_JOINTS; i++) {
    sprintf(buf,"RS.%s_trq", RobotState::joint_names[i].c_str());
    logger.add_datapoint(buf, "Nm", &(joints_torque[i]));
  }
}
