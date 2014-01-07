#include "kinematicfilter.h"
#include "Utils.hpp"

#include "drc_pelvis_defines.h"
#include "drc_hat_defines.h"
#include "drc_hat_ns_defines.h"
#include "SDModel_common.h"

// Static member
Eigen::Matrix<double,3,1> KinematicFilterEric::_gravity(0,0,-GRAVITY);
#ifdef ATLAS_ONLINE
Eigen::Matrix<double,3,1> KinematicFilterEric::_imu_lin_offset(-0.0905, -0.000004, -0.012);
Eigen::Quaterniond KinematicFilterEric::_imu_quat_offset(0.923879532511287, 0, 0, 0.382683432365090);   // IMU quaternion
#else
Eigen::Matrix<double,3,1> KinematicFilterEric::_imu_lin_offset(0.0484, 0, -0.04087);
Eigen::Quaterniond KinematicFilterEric::_imu_quat_offset(1, 0, 0, 0);   // IMU quaternion
#endif

KinematicFilterEric::KinematicFilterEric (const KinematicFilterEric & other)
{
  copy(other);
}
                                                    
KinematicFilterEric& KinematicFilterEric::operator=(const KinematicFilterEric & other)
{
  copy(other);
  return *this; 
}

KinematicFilterEric::KinematicFilterEric()
{
  _P.setIdentity();
  _Q.setIdentity();
  _R.setIdentity();
  rs = NULL;
  sdFastState = NULL;
}

KinematicFilterEric::~KinematicFilterEric() 
{
  if (rs)
    delete rs;
  if (sdFastState)
    delete []sdFastState;
} 

// Takes angular velocity and linear acc as input
void KinematicFilterEric::predictX()
{
  // pelvis to IMU offset : 0.05991527603 , 0, -0.0141
  // pelvis to COM offset : 0.011486, 0, 0.026773
  //Eigen::Matrix<double,3,1> imuoffset = Eigen::Matrix<double,3,1>::Constant(0.0484, 0, -0.04087);
  //Eigen::Matrix<double,3,1> imuoffset;
  //imuoffset << 0.0484, 0, -0.04087;
  // Gravity 
  //_gravity << 0, 0, -GRAVITY;
  // IMU acceleration in world frame without gravity (a = R^{-1}*a_imu + g)
  Eigen::Matrix<double,3,1> acc;
  acc = _q.conjugate()*_imu_quat_offset.conjugate()*Eigen::Map<const Eigen::Matrix<double,3,1> >(_a) + _gravity;
  
  // LIMIT ACCELERATION TO BE BETWEEN -5 and 5
  //double thre = 3;
  //for (int i = 0; i < 3; i++) {
  //  if(acc(i,0) > thre) acc(i,0) = thre;
  //  if(acc(i,0) < -1*thre) acc(i,0) = -1*thre;
  //}
          
  
  // IMU velocity in world frame
  //_vimu = _q*_x.block(V0,0,3,1) + _w.cross(imuoffset);
  // IMU velocity in world frame
  _vimu += timestep*acc;
  // Position
  _x.block(X0,0,3,1) += timestep*_x.block(V0,0,3,1);// + 0.5*timestep*timestep*acc; 
  // Velocity
  // V = R(-q)*(Vimu - \omega x imuoffset)
  _x.block(V0,0,3,1) = _vimu - _q*Eigen::Map<const Eigen::Matrix<double,3,1> >(_w).cross(_imu_lin_offset); 
}

void KinematicFilterEric::updateZ(int zDim = 6)
{
  _z.setZero();  
  _z.block(0,0,3,1) = _beta*foot_pos[LEFT] + (1.0 - _beta)*foot_pos[RIGHT];
  _z.block(3,0,3,1) = _beta*foot_vel[LEFT] + (1.0 - _beta)*foot_vel[RIGHT]; 
} 

void KinematicFilterEric::updateX(int zDim = 6)
{
  /*if (zDim == 6){*/
  _dx= _K* _innov;
  /*}*/
  _x += _dx;
  //Eigen::Matrix<double,3,1> imuoffset = Eigen::Matrix<double,3,1>::Constant(0.0484, 0, -0.04087);
  //Eigen::Matrix<double,3,1> imuoffset;
  //imuoffset << 0.0484, 0, -0.04087;
  _vimu = _x.block(V0,0,3,1) + _q.conjugate()*Eigen::Map<const Eigen::Matrix<double,3,1> >(_w).cross(_imu_lin_offset);
  // LIMIT z velocity
  //double thre = 0.5;
  //if (_x(V0+2,0) > thre) _x(V0+2) = thre;
  //if (_x(V0+2,0) < -1*thre) _x(V0+2) = -1*thre;
}
void KinematicFilterEric::computeInnovation(const Eigen::Matrix<double, 6, 1> & z)
{
  _innov = z - _z; 
}

void KinematicFilterEric::makeProcessJacobian ()
{
  _A.setIdentity();
  _A.block(0,3,3,3).setIdentity();
  _A.block(0,3,3,3) *= timestep;
}

void KinematicFilterEric::makeObservationJacobian ()
{
  _C.setIdentity();
}

// Ankle Related Stuff                        
void KinematicFilterEric::computeAnkleRelated()
{
  int joint[2] = {PELV_S_JOINT_L_LEG_LAX, PELV_S_JOINT_R_LEG_LAX};
  int body[2] = {PELV_S_BODY_L_FOOT, PELV_S_BODY_R_FOOT};

  if (modelType == RobotState::TYPE_HAT) {
    body[0] = HAT_S_BODY_L_FOOT;
    body[1] = HAT_S_BODY_R_FOOT;
    joint[0] = HAT_S_JOINT_L_LEG_LAX;
    joint[1] = HAT_S_JOINT_R_LEG_LAX;
  }
  
  if (modelType == RobotState::TYPE_HAT_NS) {
    body[0] = HATNS_S_BODY_L_FOOT;
    body[1] = HATNS_S_BODY_R_FOOT;
    joint[0] = HATNS_S_JOINT_L_LEG_LAX;
    joint[1] = HATNS_S_JOINT_R_LEG_LAX;
  }

  rs->makeSDFastState(_x.block(X0,0,3,1).data(), _q.coeffs().data(), _x.block(V0,0,3,1).data() , _w, _joints, _jointsd, sdFastState);
  set_state(rs->model, sdFastState);

  double com_to_ankle[3];
  double com_to_ref[3]; // Com to reference point.

  //dvec_copy(tmp, foot_contact_point_offset[RIGHT], Foot::com_to_ankle_offset, 3);
  //for (int side = LEFT; side < LR; side++)
  //  for (int i = 0; i < 3; i++) {
  //    foot_contact_point_offset[side][i] = (1.0-_weight[side])*rs.feet[side].com_to_heel_offset[i]+(_weight[side])*rs.feet[side].com_to_toe_offset[i];
  //} 

  for (int side = 0; side < 2; side++) {
    rs->model->sdgetbtj(joint[side], com_to_ankle);
		//dvec_add(com_to_ankle, feet[side].ankle_to_ref, temp, 3);
    for (int i = 0; i < 3; i++) {
      //com_to_ref[i] = com_to_ankle[i] + Foot::ankle_to_toe_offset[i];
      com_to_ref[i] = com_to_ankle[i] + Foot::ankle_to_sensor_offset[i];
    }
    dvec_copy(foot_contact_point_offset[side], com_to_ref, 3);
     
    rs->model->sdpos(body[side], foot_contact_point_offset[side], foot_pos[side].data()); 
    rs->model->sdvel(body[side], foot_contact_point_offset[side], foot_vel[side].data()); 
  }
}  

// This function computes Kss once
void KinematicFilterEric::setKss()
{
  makeProcessJacobian();
  makeObservationJacobian();
  //setQ(DSc_Q);
  //setR(DSc_R);
  computeKss(_A,_C, 6);
}

// Steady state filter, call setKss first
void KinematicFilterEric::filterOneTimeStep_ss()
{ 
  predictX();
  computeAnkleRelated(); // Compute ankle position and velocity
  updateZ(6);
  computeInnovation(_y);
  updateX(6);
  // The following steps are additional step for next step measurement
  computeFootPV(); // Compute current foot position and velocity
  //fill_measurement_buffer(); // Fill buffer
  //set_hack_footpos();
  //std::cout << "Buffer\n\n" << foot_buffer << std::endl;
  //std::cout << "Foot PV \n\n" << foot_pv.transpose() << std::endl;
  //std::cout << "Measurement \n" << _y.transpose() << std::endl;
  //std::cout << "Predicted Measurement \n" << _z.transpose() << std::endl;
  //std::cout << "Beta\n\n" << _beta << std::endl;
  //getchar();
}
/*void getQuaternion(double *q)*/
/*{*/
/*memcpy(q, _x.block(Quat0,0,4,1).data(), sizeof(double)*4); */
/*}*/

void KinematicFilterEric::getLinearV(double *v)
{
  memcpy(v, _x.block(V0,0,3,1).data(), sizeof(double)*3); 
} 

void KinematicFilterEric::getPosition(double *x)
{
  memcpy(x, _x.block(X0,0,3,1).data(), sizeof(double)*3); 
}  

void KinematicFilterEric::getFootPose(int side, double *x)
{
  if (side == LEFT)
    memcpy(x, foot_registered.block(0,0,3,1).data(), sizeof(double)*3); 
  if (side == RIGHT)
    memcpy(x, foot_registered.block(6,0,3,1).data(), sizeof(double)*3); 
}  

void KinematicFilterEric::makeInputs(double *q, double *w, double *a, double *joints, double *jointsd)
{
  // dvec_cooy(_q, q, 4);
  _q = Eigen::Map<const Eigen::Matrix<double,4,1> >(q); 
  dvec_copy(_w, w, 3);
  dvec_copy(_a, a, 3);
  dvec_copy(_joints, joints, N_JOINTS);
  dvec_copy(_jointsd, jointsd, N_JOINTS);
}

void KinematicFilterEric::getContactState (int cs)
{
  _contact_state_full = cs;
  _contact_state = cs;
  // The following are defined in drc_common_defines.h
  if (cs == DSl) 
    _contact_state = DSc;
  if (cs == DSr)
    _contact_state = DSc;
}


void KinematicFilterEric::readParams(std::string &nameQ, std::string &nameR)
{
  std::ifstream in;
  in.open(nameQ.c_str());
    //std::ifstream in ("cmu_config/filter_param/DSc_Q2");
  if(in.is_open())
    in >> _Q;    
  in.close(); 
  in.open(nameR.c_str());
    //std::ifstream in ("cmu_config/filter_param/DSc_Q2");
  if(in.is_open())
    in >> _R;    
  in.close();
}
// ROS
void KinematicFilterEric::initKF(int type, double dt, const double root_pos[3], const double root_vel[3])
{
  modelType = type;
  buffer_first_time = true;
  contact_change_flag = true;

  if (modelType == RobotState::TYPE_PELVIS) {
    if (!rs) {
      rs = new PelvRobotState();
      sdFastState = new double[PELV_N_SDFAST_STATE];
    }
  }
  else if (modelType == RobotState::TYPE_HAT) {
    if (!rs) {
    rs = new HatRobotState();
    sdFastState = new double[HAT_N_SDFAST_STATE];
    }
  }
  else if (modelType == RobotState::TYPE_HAT_NS) {
    if (!rs) {
      rs = new HatNSRobotState();
      sdFastState = new double[HATNS_N_SDFAST_STATE];
    }
  }

  timestep = dt;
  
  Eigen::Matrix<double,6,6> P0;
  P0.setIdentity();
  P0 *= 0.01;
  setP(P0);
  
  dvec_copy(_x.block(X0,0,3,1).data(), root_pos, 3);
  dvec_copy(_x.block(V0,0,3,1).data(), root_vel, 3);
  _vimu = _x.block(V0,0,3,1); // This is not quite accurate, but only one time step, so does not matter
  setKss(); 

  // The following steps are additional step for next step measurement
  computeFootPV(); // Compute current foot position and velocity
  //fill_measurement_buffer(); // Fill buffer
  set_hack_footpos();
  /*
     timestep = RS.timeStep;
     dvec_copy(_x.block(0,0,3,1).data(), RS.root, 3);
     dvec_copy(_x.block(3,0,3,1).data(), RS.rootd, 3);
     */
}

void KinematicFilterEric::fill_measurement_buffer()
{
  if (buffer_first_time)
  {
    buffer_first_time = false;
    buffer_index = 0;
    foot_buffer.setZero();
    for (int i = 0; i < buffer_size; i++)
      foot_buffer.col(i) = foot_pv;
    _beta = 0.5; // Initialize beta here
  }
  else
  {
    if (buffer_index == buffer_size)
      buffer_index = 0;
    foot_buffer.col(buffer_index) = foot_pv;
    buffer_index++;
  }
}

void KinematicFilterEric::computeFootPV()
{
  computeAnkleRelated();
  foot_pv.block(0,0,3,1) = foot_pos[LEFT];
  foot_pv.block(3,0,3,1) = foot_vel[LEFT];
  foot_pv.block(6,0,3,1) = foot_pos[RIGHT];
  foot_pv.block(9,0,3,1) = foot_vel[RIGHT];
}


void KinematicFilterEric::set_hack_footpos()
{
  if (buffer_first_time)
  {
    buffer_first_time = false;
    _beta = 0.5; // Initialize beta here
  }
  if (contact_change_flag )
  {
    foot_registered = foot_pv;
    contact_change_flag = false;
  }
}

void KinematicFilterEric::makeMeasurement(double left_fz, double right_fz)
{
  /*
     double tmp = 0.0;
     double e = 0.9; // LPF parameter
  // Compute weight distribution based on foot force sensor data
  if (fabs(left_fz + right_fz) > 1e0)
  tmp = left_fz / (left_fz + right_fz); 
  _beta = e*_beta + (1.0 - e)*tmp;
  */

  switch (_contact_state_full){
    case DSc: // z = fkl, vel_l, fkr, vel_r
      _beta = 0.5;
      break;
    case SSL: // z = fkl, vel_l
    //case DSl:
    case DSr:
      _beta = 1.0;
      break;
    case SSR: // z =fkr, vel_r
    //case DSr:
    case DSl:
      _beta = 0.0;
      break;
    default:
      printf("Unknow contact state %d\n", _contact_state_full);
      exit(-1);
  } 
 
  if (!(_previous_contact_state == _contact_state_full))
  {
    _previous_contact_state = _contact_state_full;
    contact_change_flag = true;
    set_hack_footpos();
  }

  if (sqrt(left_fz*left_fz+right_fz*right_fz) < 50) // Total z force small
  {
    contact_change_flag = true; // Just update foot position until the robot drops from the air
    set_hack_footpos();
  }
  
  // Compute measurement
  //_ytemp = foot_buffer.rowwise().sum()*(1./(double) buffer_size);
  //if (contact_state_change)
  _ytemp = foot_registered;  
  _y = _beta*_ytemp.block(0,0,6,1) + (1.0 - _beta)*_ytemp.block(6,0,6,1);
  // Set measured velocity to zero
  _y.block(3,0,3,1).setZero();
}

void KinematicFilterEric::setX (const double * x, int len)
{
  assert(len == 6); 
  _x = Eigen::Map<const Eigen::Matrix<double,6,1> >(x);
}
// ============================================================================
/// @brief set internal state by Eigen matrix 
///
/// @param x Eigen Column Matrix
// ============================================================================
void KinematicFilterEric::setX (const Eigen::Matrix<double, 6, 1> & x){ _x = x; }
// ============================================================================
/// @brief set covariance
///
/// @param P Eigen Matrix
// ============================================================================
void KinematicFilterEric::setP (const Eigen::Matrix<double, 6, 6> & P) { _P = P; }
// ============================================================================
/// @brief set process covariance
///
/// @param Q Eigen Matrix
// ============================================================================
void KinematicFilterEric::setQ (const Eigen::Matrix<double, 6, 6> & Q) { _Q = Q; }
// ============================================================================
/// @brief set observation covariance
///
/// @param R Eigen Matrix
// ============================================================================
void KinematicFilterEric::setR (const Eigen::Matrix<double, 6, 6> & R)  { _R = R; }

// ============================================================================
/// @brief get state and save it to x[len]
///
/// @param x double array
/// @param len length of x
// ============================================================================
void KinematicFilterEric::getX(double * x, int len)
{ 
  assert(len == 6); 
  memcpy(x, _x.data(), sizeof(double)*len); 
}  
// ============================================================================
/// @brief get state in Eigen
///
/// @return Eigen Matrix
// ============================================================================
const Eigen::Matrix<double,6,1> & KinematicFilterEric::getX() const { return _x; }             
// ============================================================================
/// @brief get predicted observation in Eigen
///
/// @return Eigen Matrix
// ============================================================================
const Eigen::Matrix<double,6,1> & KinematicFilterEric::getZ() const { return _z; }
// ============================================================================
/// @brief get innovation vector
///
/// @return Eigen Matrix
// ============================================================================
const Eigen::Matrix<double,6,1> & KinematicFilterEric::getInnov() const { return _innov; }
// ============================================================================
/// @brief get process covariance
///
/// @return Eigen Matrix
// ============================================================================
const Eigen::Matrix<double,6,6> & KinematicFilterEric::getQ() const { return _Q; } 
// ============================================================================
/// @brief get observation covariance
///
/// @return Eigen Matrix
// ============================================================================
const Eigen::Matrix<double,6,6> & KinematicFilterEric::getR() const { return _R; }  

void KinematicFilterEric::computeKss(const Eigen::Matrix<double,6,6> &A, const Eigen::Matrix<double,6,6> &C, int zDim = 6)
{

  Eigen::Matrix<double,6,6> B = C.transpose();
  // Eigen::Matrix<double,6,6> P;
  dare(A.transpose(), B, _P, zDim); // A^T is used here
  _K.setZero();
  Eigen::Matrix<double,6,6> PB; 
  Eigen::Matrix<double,6,6> BtPB_R; 
  if (zDim == 6){
    PB = _P * B;
    BtPB_R = B.transpose() * PB + _R; 
    _K = PB* BtPB_R.inverse();
  }
  else
  {
    PB.topLeftCorner(6,zDim) = _P * B.topLeftCorner(6,zDim);
    BtPB_R.topLeftCorner(zDim,zDim) = B.topLeftCorner(6,zDim).transpose() * PB.topLeftCorner(6,zDim) + _R.topLeftCorner(zDim,zDim);
    _K.topLeftCorner(6,zDim) = PB.topLeftCorner(6,zDim)* BtPB_R.topLeftCorner(zDim,zDim).inverse();  
  }
}

void KinematicFilterEric::RodriguesFormula (const Eigen::Matrix<double,3,1> & u, Eigen::Matrix<double,3,3> & EA) 
{
  EA.setIdentity();
  double alpha = u.squaredNorm();
  Eigen::Matrix<double,3,3> A; // Cross product matrix
  A << 0, -u(2), u(1),
    u(2), 0, -u(0),
    -u(1), u(0), 0;
  if (alpha > 1e-12)
    EA += sin(alpha)/alpha*A + (cos(alpha)-1)/(alpha*alpha)*A*A; 
}

void KinematicFilterEric::copy(const KinematicFilterEric & other) 
{
  _x = other._x;  
  _z = other._z;
  _innov = other._innov;
  _P = other._P;
  _Q = other._Q;
  _R = other._R;
  _K = other._K;
  _S = other._S;
}

void KinematicFilterEric::dare(const Eigen::Matrix<double,6,6> &A, const Eigen::Matrix<double,6,6> &B, Eigen::Matrix<double,6,6> &P,int zDim = 6) 
{                                      
  Eigen::Matrix<double,6,6> Ainv = A.inverse();
  Eigen::Matrix<double,6,6> ABRB; 
  if (zDim == 6)
  {
    ABRB = Ainv * B * _R.llt().solve(B.transpose());
  }
  else{
    ABRB = Ainv * B.topLeftCorner(6,zDim) * _R.topLeftCorner(zDim,zDim).llt().solve(B.topLeftCorner(6,zDim).transpose());
  }                            
  Eigen::Matrix<double,2*6,2*6> Z;
  Z.block(0,0,6,6) = Ainv;
  Z.block(0,6,6,6) = ABRB;
  Z.block(6,0,6,6) = _Q * Ainv;
  Z.block(6,6,6,6) = A.transpose() + _Q * ABRB;

  Eigen::ComplexEigenSolver <Eigen::Matrix<double,2*6,2*6> > ces;
  ces.compute(Z);

  Eigen::Matrix<std::complex<double>,2*6,1> eigVal = ces.eigenvalues();
  Eigen::Matrix<std::complex<double>,2*6,2*6> eigVec = ces.eigenvectors();

  Eigen::Matrix<std::complex<double>,2*6,6> unstableEigVec;

  int ctr = 0;
  for (int i = 0; i < 2*6; i++) {
    if (eigVal(i).real()*eigVal(i).real() + eigVal(i).imag()*eigVal(i).imag() > 1) {
      unstableEigVec.col(ctr) = eigVec.col(i);
      ctr++;
      if (ctr > 6)
        break;
    }
  }

  Eigen::Matrix<std::complex<double>,6,6> U21inv = unstableEigVec.block(0,0,6,6).inverse();
  Eigen::Matrix<std::complex<double>,6,6> PP = unstableEigVec.block(6,0,6,6) * U21inv;

  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) {
      P(i,j) = PP(i,j).real();
    }
  }
} 
