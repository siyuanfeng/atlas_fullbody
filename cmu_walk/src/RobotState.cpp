#include "RobotState.h"
#include <stdio.h>
#include <stdlib.h>
#include "sdlib.h"
#include <string.h>
#include <fstream>
#include "Utils.hpp"
#include "Eigen_utils.hpp"

#ifndef ATLAS_ONLINE
const double RobotState::jointsCFMDamping[N_JOINTS] = 
{
  /*
     10.0, 10.0, 10.0, 0.1,
     10.0, 10.0, 10.0, 0.1, 0.1, 0.1,
     10.0, 10.0, 10.0, 0.1, 0.1, 0.1,
     0.1, 0.1, 0.1, 0.1, 0.1, 0.1,
     0.1, 0.1, 0.1, 0.1, 0.1, 0.1
     */
  10.0, 10.0, 10.0, 1,
  1.0, 1.0, 1.0, 1, 1, 1,
  1.0, 1.0, 1.0, 1, 1, 1,
  1, 1, 1, 1, 1, 1,
  1, 1, 1, 1, 1, 1
}; 
#endif

/*
   const double RobotState::jointsLimit[2][N_JOINTS] = 
   {
   {
   -0.663225, -0.610691, -0.698132, 
   -0.602139,
   -0.174533, -0.523599, -1.72072, 0.0, -1, -0.8,
   -1.22173, -0.523599, -1.72072, 0.0, -1, -0.8,
   -1.5708, -1.5708, 0.0, 0.0, 0.0, -1.1781,
   -1.5708, -1.5708, 0.0, -2.35619, 0, -1.1781
   },
   {
   0.663225, 0.438427, 0.698132, 
   1.14494,
   1.22173, 0.523599, 0.524821, 2.38569, 0.7, 0.8,
   0.174533, 0.523599, 0.524821, 2.38569, 0.7, 0.8,
   0.785398, 1.5708, 3.14159, 2.35619, 3.14159, 1.1781,
   0.785398, 1.5708, 3.14159, 0.0, 3.14159, 1.1781
   }
   };
   */



const double RobotState::jointsLimit[2][N_JOINTS] = 
{
  {
    -0.4, -0.64, -0.72,
    -0.61,
    -0.17, -0.34, -1.75, 0, -1.0, -0.61,
    -1.21, -0.46, -1.75, 0, -1.0, -0.61,
    -1.57, -1.57, 0, -0.04, 0.01, -1.18,
    -1.56, -1.57, 0.01, -2.32, 0, -1.18
  },
  {
    0.95, 0.482, 0.69,
    1.15,
    1.22, 0.53, 0.54, 2.35, 0.72, 0.61,
    0.18, 0.35, 0.52, 2.39, 0.72, 0.61,
    0.78, 1.57, 3.14, 2.31, 3.15, 1.17,
    0.79, 1.57, 3.15, 0, 3.15, 1.18
  }
};

/*
const double RobotState::torqueLimit[2][N_JOINTS] = 
{
  {
    -124.016, -206.843, -500.0,
    -5.0,
    -110.0, -180.0, -260.0, -520.0, -700.0, -90.0,
    -110.0, -180.0, -260.0, -520.0, -700.0, -90.0,
    -212.0, -170.0, -114.0, -114.0, -114.0, -60.0,
    -212.0, -170.0, -114.0, -114.0, -114.0, -60.0
  },
  {
    124.016, 206.843, 500.0,
    5.0,
    110.0, 180.0, 260.0, 520.0, 700.0, 90.0,
    110.0, 180.0, 260.0, 520.0, 700.0, 90.0,
    212.0, 170.0, 114.0, 114.0, 114.0, 60.0,
    212.0, 170.0, 114.0, 114.0, 114.0, 60.0
  }
};
*/

const std::string RobotState::joint_names[ N_JOINTS ] = { 
  "back_lbz", "back_mby", "back_ubx", "neck_ay",
  "l_leg_uhz", "l_leg_mhx", "l_leg_lhy", "l_leg_kny", "l_leg_uay", "l_leg_lax",
  "r_leg_uhz", "r_leg_mhx", "r_leg_lhy", "r_leg_kny", "r_leg_uay", "r_leg_lax",
  "l_arm_usy", "l_arm_shx", "l_arm_ely", "l_arm_elx", "l_arm_uwy", "l_arm_mwx",
  "r_arm_usy", "r_arm_shx", "r_arm_ely", "r_arm_elx", "r_arm_uwy", "r_arm_mwx" };

const std::string RobotState::body_names[ N_BODIES ] = {
  "pelvis", "ltorso", "mtorso", "utorso", "head",
  "l_uglut", "l_lglut", "l_uleg", "l_lleg", "l_talus", "l_foot",
  "r_uglut", "r_lglut", "r_uleg", "r_lleg", "r_talus", "r_foot",
  "l_clav", "l_scap", "l_uarm", "l_larm", "l_farm", "l_hand",
  "r_clav", "r_scap", "r_uarm", "r_larm", "r_farm", "r_hand"
};

//A. The robot zero is specified by:
//X: center of hip Z joint 
//Y: center of the pelvis 
//Z: center of hip X joint 
const double RobotState::root2tfRoot[3] = {-0.0113715, 0, -0.0268706};

RobotState::RobotState() 
{
  //setInternalPointers();
  fillZeros();
  // model->sdinit();
} 

/*
   RobotState &RobotState::operator=(const RobotState &other) 
   {
   if (this == &other || _type != other._type)  
   return *this;

   time = other.time;
   ctrl_dt = other.ctrl_dt;
   timeStep = other.timeStep;
   dvec_copy(root, other.root, 6);
   dvec_copy(rootd, other.rootd, 6);
   dvec_copy(rootdd, other.rootdd, 6);
   dvec_copy(root_b_w, other.root_b_w, 3);
   dvec_copy(root_b_wd, other.root_b_wd, 3);

//dvec_copy(rootq, other.rootq, QQ);
rootq = other.rootq;
dvec_copy(joints, other.joints, N_JOINTS);
dvec_copy(jointsd, other.jointsd, N_JOINTS);
dvec_copy(jointsdd, other.jointsdd, N_JOINTS);
dvec_copy(joints_torque, other.joints_torque, N_JOINTS);

for (int i = 0; i < N_LOCATIONS; i++)
dvec_copy(locations[i], other.locations[i], XYZ);

for (int i = 0; i < LR; i++)
feet[i] = other.feet[i];

utorsoq = other.utorsoq;
dvec_copy(utorsow, other.utorsow, 3);

dvec_copy(react, other.react, 12);
dvec_copy(com, other.com, 3);
dvec_copy(comd, other.comd, 3);
dvec_copy(comdd, other.comdd, 3);

// dvec_copy(sdFastState, other.sdFastState, PELV_N_SDFAST_STATE);

for (int i = 0; i < LR; i++) {
J[i] = other.J[i];
Jd[i] = other.Jd[i];
}
Jc = other.Jc;
Jcd = other.Jcd;
Jtorso = other.Jtorso;
Jtorsod = other.Jtorsod;

M = other.M;
I = other.I;
Itorso = other.Itorso;

nonLin = other.nonLin;

m = other.m;
dvec_copy(mom, other.mom, 3);
dvec_copy(angMom, other.angMom, 3);
KE = other.KE;
contactState = other.contactState;
contact[0] = other.contact[0];
contact[1] = other.contact[1];

return *this;
}
*/

Eigen::Vector3d RobotState::getFootCenter() const {
  Eigen::Vector3d c((feet[LEFT].w_sensor_pos[XX]+feet[RIGHT].w_sensor_pos[XX])/2.0, (feet[LEFT].w_sensor_pos[YY]+feet[RIGHT].w_sensor_pos[YY])/2.0, (feet[LEFT].w_sensor_pos[ZZ]+feet[RIGHT].w_sensor_pos[ZZ])/2.0);
  return c;
}

Eigen::Vector3d RobotState::getFootCenterLower() const {
  Eigen::Vector3d c((feet[LEFT].w_sensor_pos[XX]+feet[RIGHT].w_sensor_pos[XX])/2.0, (feet[LEFT].w_sensor_pos[YY]+feet[RIGHT].w_sensor_pos[YY])/2.0, std::min(feet[LEFT].w_sensor_pos[ZZ], feet[RIGHT].w_sensor_pos[ZZ]));
  return c;
}

void RobotState::checkContact() {
	for(int s = 0; s < 2; s++) {
		if(footContact[s]) {	//if already recording contact
			if(feet[s].w_F[ZZ] < 80) 	contactCount[s]++;
			else						contactCount[s] = 0;
		}
		else {					//if already recording not contact
			if(feet[s].w_F[ZZ] > 100)	contactCount[s]++;
			else						contactCount[s] = 0;
		}

		if(contactCount[s] >= 5) {		//time for a contact change
			footContact[s] = !footContact[s];
			contactCount[s] = 0;
		}
	}
}

void RobotState::fillZeros() 
{
  time = 0;
  ctrl_dt = 0;
  timeStep = 1e-3;

  cop[0] = cop[1] = 0;
  contactCount[LEFT] = contactCount[RIGHT] = 0;
  footContact[LEFT] = footContact[RIGHT] = false;
  //for(int i = 0; i < 12; i++)
  //	react[i] = 0;
  //for(int i = 0; i < PELV_N_SDFAST_STATE; i++) 
  //  sdFastState[i] = 0;

  for(int j = 0; j < 6; j++) {
    root[j] = 0;
    rootd[j] = 0;
    rootdd[j] = 0;
  }

  for (int j = 0; j < 3; j++) {
    root_b_w[j] = 0;
    root_b_wd[j] = 0;
  }

  rootq = Eigen::Quaterniond::Identity();

  for (int i = 0; i < N_JOINTS; i++) {
    joints[i] = 0;
    jointsd[i] = 0;
    jointsdd[i] = 0;
    joints_torque[i] = 0;
  }

  for(int i = 0; i < 3; i++) {
    com[i] = 0;
    comd[i] = 0;
    comdd[i] = 0;
    mom[i] = 0;
    angMom[i] = 0;
  }

  /*
     for (int i = 0; i < LR; i++) {
     J[i] = Eigen::Matrix<double,PELV_N_SDFAST_U,6,Eigen::RowMajor>::Zero();
     Jd[i] = Eigen::Matrix<double,PELV_N_SDFAST_U,6,Eigen::RowMajor>::Zero();
     }
     Jc = Eigen::Matrix<double,PELV_N_SDFAST_U,3,Eigen::RowMajor>::Zero(); 
     Jcd = Eigen::Matrix<double,PELV_N_SDFAST_U,3,Eigen::RowMajor>::Zero();

     Jtorso = Eigen::Matrix<double,PELV_N_SDFAST_U,6,Eigen::RowMajor>::Zero();
     Jtorsod = Eigen::Matrix<double,PELV_N_SDFAST_U,6,Eigen::RowMajor>::Zero();

     M = Eigen::Matrix<double,PELV_N_SDFAST_U,PELV_N_SDFAST_U,Eigen::RowMajor>::Zero();
     I = Eigen::Matrix<double,3,3,Eigen::RowMajor>::Zero();
     Itorso = Eigen::Matrix<double,3,3,Eigen::RowMajor>::Zero();
     */

  dvec_set(handForces[LEFT], 0, 6);
  dvec_set(handForces[RIGHT], 0, 6);

  m = 0;
  KE = 0;
  contactState = DSc;
}

/*
   void RobotState::makePelvSdfastState(const double pos[3], const Eigen::Quaterniond &q, const double vel[3], const double w[3], const double j[N_JOINTS], const double jd[N_JOINTS], double sdfast_state[PELV_N_SDFAST_STATE])
//void RobotState::makePelvSdfastState(const double pos[3], const double q[4], const double vel[3], const double w[3], const double j[N_JOINTS], const double jd[N_JOINTS], double sdfast_state[PELV_N_SDFAST_STATE])
{
for (int i = 0; i < PELV_N_SDFAST_STATE; i++)
sdfast_state[i] = 0;

// copy joint pos and vel
memcpy(sdfast_state+6, j, sizeof(double)*N_JOINTS);
memcpy(sdfast_state+PELV_N_SDFAST_Q+6, jd, sizeof(double)*N_JOINTS);

// get position and vel 
memcpy(sdfast_state, pos, sizeof(double)*XYZ);
sdfast_state[PELV_S_Q0] = q.x();
sdfast_state[PELV_S_Q1] = q.y();
sdfast_state[PELV_S_Q2] = q.z();
sdfast_state[PELV_S_Q3] = q.w();

memcpy(sdfast_state+PELV_N_SDFAST_Q, vel, sizeof(double)*XYZ);
memcpy(sdfast_state+PELV_N_SDFAST_Q+3, w, sizeof(double)*XYZ);
}
*/

void RobotState::computeSDFvars(const double pos[3], const double q[4],   
    const double vel[3], const double w[3],
    const double j[N_JOINTS], const double jd[N_JOINTS])
{
  dvec_copy(root, pos, 3);
  dvec_copy(rootd, vel, 3);
  rootq = Eigen::Quaterniond(q[3], q[0], q[1], q[2]);
  dvec_copy(root_b_w, w, 3);
  dvec_copy(joints, j, N_JOINTS);
  dvec_copy(jointsd, jd, N_JOINTS);
  computeSDFvars();
}

inline void filtUpdate(double &oldVal, double newVal, double a) {	oldVal = (1-a)*oldVal + a*newVal; }
inline void filtUpdate(double *oldArray, const double *newArray, double a, int N) {	for(int i = 0; i < N; i++)		filtUpdate(oldArray[i], newArray[i], a);}

void RobotState::computeSDFvarsFilt(const double pos[3], const double q[4],
    const double vel[3], const double w[3],
    const double j[N_JOINTS], const double jd[N_JOINTS], double filtA)
{
  filtUpdate(root, pos, filtA, 3);
  filtUpdate(rootd, vel, filtA, 3);
  Eigen::Quaterniond qNew(q[3], q[0], q[1], q[2]);
  rootq = mySlerp(rootq, qNew, filtA);
  filtUpdate(root_b_w, w, filtA, 3);
  filtUpdate(joints, j, filtA, N_JOINTS);
  filtUpdate(jointsd, jd, filtA, N_JOINTS);

  computeSDFvars();
}

/*
   void RobotState::computeSDFvars()
   {	
//storePrevs();

makePelvSdfastState(root, rootq, rootd, root_b_w, joints, jointsd, sdFastState);
set_state(&model-> sdFastState);

double temp[6];
double zeros[3] = {0, 0, 0};
int body[2] = {PELV_S_BODY_L_FOOT, PELV_S_BODY_R_FOOT};
//momentem and KE
model->sdmom( mom, angMom, &KE);
double dircos[3][3];
double qq[4];

for (int side = 0; side < 2; side++) {
//foot positions and jacobians
body_position(&model-> NULL, body[side], feet[side].ref_offset, feet[side].w_sensor_pos, J[side].data());
//foot quaternion
model->sdorient(body[side], dircos);
model->sddc2quat(dircos, qq, qq+1, qq+2, qq+3);
feet[side].w_q = Eigen::Quaterniond(qq[3], qq[0], qq[1], qq[2]).normalized();
//foot velocities
model->sdvel(body[side], feet[side].ref_offset, feet[side].w_sensor_vel);
//foot angular velocities
model->sdangvel(body[side], feet[side].b_w);
model->sdtrans(body[side], feet[side].b_w, -1, &(feet[side].w_sensor_vel[3]));

// foot tip
for (int i = 0; i < 4; i++) {
model->sdpos(body[side], feet[side].tip_offset[i], feet[side].vertices[i]);
model->sdvel(body[side], feet[side].tip_offset[i], feet[side].verticesd[i]);
}
}

model->sdorient(PELV_S_BODY_PELVIS, dircos);
model->sddc2ang(dircos, root+3, root+4, root+5);

// fill out various body part locations
double offset[3];
for (int i = 1; i < N_BODIES; i++) {
model->sdgetbtj(i, offset);
model->sdpos(i, offset, &(locations[i-1][0]));
}

//mass, com position, and total moment of inertia
model->sdsys(&m, com, (double (*)[3])I.data());

//comd from momentum
for(int i = 0; i < 3; i++)
comd[i] = mom[i]/m;		//compute comd from momentum

// pelvis angular velocity in world frame
model->sdangvel(PELV_S_BODY_PELVIS, temp);
model->sdtrans(PELV_S_BODY_PELVIS, temp, -1, rootd+3);

// utorso orientation
body_position(&model-> NULL, PELV_S_BODY_UP_TORSO, zeros, temp, Jtorso.data());
model->sdorient(PELV_S_BODY_UP_TORSO, dircos);
model->sddc2quat(dircos, qq, qq+1, qq+2, qq+3);
utorsoq = Eigen::Quaterniond(qq[3], qq[0], qq[1], qq[2]).normalized();
model->sdangvel(PELV_S_BODY_UP_TORSO, temp);
model->sdtrans(PELV_S_BODY_UP_TORSO, temp, -1, utorsow);

//mass matrix
model->sdmassmat(M.data());

//com jacobian
com_jacobian(&model-> Jc.data());

//com inertia
model->sdgetiner(PELV_S_BODY_PELVIS, (double (*)[3])Itorso.data());

// non lin part
model->sdfrcmat(nonLin);
// add damping torques
for (int i = 0; i < N_JOINTS; i++) {
  nonLin[i+6] += -1 * jointsCFMDamping[i] * jointsd[i];
}

// compute Jd and Jcd by integrating one step to get next J
///////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix<double,PELV_N_SDFAST_U,6,Eigen::RowMajor> J2[LR];
Eigen::Matrix<double,PELV_N_SDFAST_U,3,Eigen::RowMajor> Jc2;
Eigen::Matrix<double,PELV_N_SDFAST_U,6,Eigen::RowMajor> Jtorso2;

double qdot[PELV_N_SDFAST_Q];
model->sdu2qdot(&sdFastState[PELV_N_SDFAST_Q], qdot);
// set new state
for (int i = 0; i < PELV_N_SDFAST_Q; i++)
sdFastState[i] += qdot[i] * timeStep; 
set_state(&model-> sdFastState);

// foot positions and jacobians
for (int side = 0; side < 2; side++) {
  body_position(&model-> NULL, body[side], feet[side].ref_offset, NULL, J2[side].data());
  Jd[side] = (J2[side] - J[side]) / timeStep;
}
// com
com_jacobian(&model-> Jc2.data());
Jcd = (Jc2 - Jc) / timeStep;

// utorso
body_position(&model-> NULL, PELV_S_BODY_UP_TORSO, zeros, temp, Jtorso2.data());
Jtorsod = (Jtorso2 - Jtorso) / timeStep;

// restore
for (int i = 0; i < PELV_N_SDFAST_Q; i++)
sdFastState[i] -= qdot[i] * timeStep; 
set_state(&model-> sdFastState); 
///////////////////////////////////////////////////////////////////////////////////

model->sdprinterr(stderr);
sdclearerr();
}
*/

void RobotState::rotateToFootForward(int side) {
	double footEA[3];
	quat2EA(feet[side].w_q, footEA);
	double yaw = footEA[2];

	double footPos[3];
	for(int i = 0; i < 3; i++)	footPos[i] = feet[side].w_sensor_pos[i];

	Eigen::Quaterniond dR = EA2quat(0,0,-yaw);
	rootq = dR*rootq;

	computeSDFvars();

	for(int i = 0; i < 3; i++)	root[i] += footPos[i]-feet[side].w_sensor_pos[i];

	computeSDFvars();
}

void RobotState::rotateAroundFoot(int side, double yaw) {
	double footPos[3];
	for(int i = 0; i < 3; i++)	footPos[i] = feet[side].w_sensor_pos[i];

	Eigen::Quaterniond dR = EA2quat(0,0,yaw);
	rootq = dR*rootq;

	computeSDFvars();

	for(int i = 0; i < 3; i++)	root[i] += footPos[i]-feet[side].w_sensor_pos[i];

	computeSDFvars();
}

void RobotState::rotateAroundFoot(double wl, double yaw) {
	double footPos[3];
	for(int i = 0; i < 3; i++)	footPos[i] = wl*feet[LEFT].w_sensor_pos[i]+(1-wl)*feet[RIGHT].w_sensor_pos[i];

	Eigen::Quaterniond dR = EA2quat(0,0,yaw);
	rootq = dR*rootq;

	computeSDFvars();

	for(int i = 0; i < 3; i++)	root[i] += footPos[i]-(wl*feet[LEFT].w_sensor_pos[i]+(1-wl)*feet[RIGHT].w_sensor_pos[i]);

	computeSDFvars();
}

void RobotState::reset() {
  for (int i = 0; i < XYZ; i++) {
    comd[i] = 0.0;
    //wd[i] = 0.0;
  }
}

