/*
 * =====================================================================================
 *
 *       Filename:  LipmWalkingCon.cpp
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  03/26/2013 04:01:38 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  YOUR NAME (), 
 *        Company:  
 *
 * =====================================================================================
 */


#include "LipmWalkingCon.h"
#include "spline.h"
#include <algorithm>
#include "Eigen_utils.hpp"
#include <map>

#define __WALKCON_LIPM_TIME_STEP      0.003

LipmWalkingCon::LipmWalkingCon()
{
  buildLookup();

  _inited = false;
  _Nsteps = 3;

  lipm_step_duration = 0.6;
  lipm_step_height = 0.15;
  lipm_step_ds_duration = 0.05;
  lipm_zmp_x_dsc = -0.13;
  lipm_zmp_x_down = -0.2;
  lipm_zmp_x_up = -0.05;
  lipm_zmp_x_normal = -0.13;
  lipm_down_thres = 0.02;
  lipm_up_thres = 0.02;

  lipm_flat_swing_scale = 0.5;
  lipm_down_swing_scale = 0.8;
  lipm_up_swing_scale = 0.2;

  lipm_z_d_normal = 0.85;
  lipm_z_d_up = 0.85;
  lipm_z_d_down = 0.85;

  fd_joint_p = 100;
  fd_joint_d = 10;

  dvec_set(fd_com_p, 100, 3);
  dvec_set(fd_com_d, 10, 3);
  dvec_set(fd_com_p+3, 150, 3);
  dvec_set(fd_com_d+3, 10, 3);
  
  dvec_set(fd_foot_p, 150, 3);
  dvec_set(fd_foot_d, 10, 3);
  dvec_set(fd_foot_p+3, 150, 3);
  dvec_set(fd_foot_d+3, 10, 3);

  fd_com_p[ZZ] = 200;
  
  fd_toeoff_y = 120;
  
  /*
  // ik gains
  ik_joint_p = 1;
  for (int i = 0; i < 3; i++) {
    ik_com_p[i] = 1;
    ik_com_p[i+3] = 1;
    ik_foot_p[i] = 1;
    ik_foot_p[i+3] = 1;
  }
  */

  no_toeoff_sqr_vel_thresh = 0.2;
  lipm_up_torso_pitch = 0.15;
  
  /*
  lipm_Q = 1e-2 * Eigen::Matrix<double,6,6>::Identity();
  lipm_R = 1e1 * Eigen::Matrix<double,3,3>::Identity();
  lipm_Q(ZZ, ZZ) = 1e1;
  lipm_R(ZZ, ZZ) = 1e-6;
  */
  
  lipm_Q.block(0,0,3,3) = 1e-4 * Eigen::Matrix<double,3,3>::Identity();
  lipm_Q.block(3,3,3,3) = 1e-2 * Eigen::Matrix<double,3,3>::Identity();
  lipm_R = 1e0 * Eigen::Matrix<double,3,3>::Identity();
  lipm_Q(ZZ, ZZ) = 1e1;
  lipm_R(ZZ, ZZ) = 1e-6;  
  
  _neck_ay_d = 0.6;
}

void LipmWalkingCon::buildLookup()
{
  _conName = "LWC";

  _lookup["lipm_step_duration"] = &lipm_step_duration;
  _lookup["lipm_step_height"] = &lipm_step_height;
  _lookup["lipm_step_ds_duration"] = &lipm_step_ds_duration;
  _lookup["lipm_zmp_x_dsc"] = &lipm_zmp_x_dsc;
  _lookup["lipm_zmp_x_down"] = &lipm_zmp_x_down;
  _lookup["lipm_zmp_x_up"] = &lipm_zmp_x_up;
  _lookup["lipm_zmp_x_normal"] = &lipm_zmp_x_normal;
  _lookup["lipm_down_thres"] = &lipm_down_thres;
  _lookup["lipm_up_thres"] = &lipm_up_thres;
  _lookup["fd_joint_p"] = &fd_joint_p;
  _lookup["fd_joint_d"] = &fd_joint_d;
  //_lookup["ik_joint_p"] = &ik_joint_p;
  
  _lookup["lipm_z_d_up"] = &lipm_z_d_up;
  _lookup["lipm_z_d_down"] = &lipm_z_d_down;
  _lookup["lipm_z_d_normal"] = &lipm_z_d_normal;
  _lookup["lipm_up_torso_pitch"] = &lipm_up_torso_pitch;

  _lookup["lipm_flat_swing_scale"] = &lipm_flat_swing_scale;
  _lookup["lipm_up_swing_scale"] = &lipm_up_swing_scale;
  _lookup["lipm_down_swing_scale"] = &lipm_down_swing_scale;

  std::string name;
  for (int i = 0; i < 6; i++) {
    name = std::string("fd_com_p_");
    name.append(boost::to_string(i));
    _lookup[name] = fd_com_p+i;
    name = std::string("fd_com_d_");
    name.append(boost::to_string(i));
    _lookup[name] = fd_com_d+i;
    name = std::string("fd_foot_p_");
    name.append(boost::to_string(i));
    _lookup[name] = fd_foot_p+i;
    name = std::string("fd_foot_d_");
    name.append(boost::to_string(i));
    _lookup[name] = fd_foot_d+i;
  }

  _lookup["fd_toeoff_y"] = &fd_toeoff_y;
  _lookup["no_toeoff_sqr_vel_thresh"] = &no_toeoff_sqr_vel_thresh;
}

LipmWalkingCon::~LipmWalkingCon()
{
  stop(); 
}

void LipmWalkingCon::stop()
{
  if (!_inited)
    return;
  // stop lipm thread
  {
    boost::mutex::scoped_lock lock(_lipmLock);
    _lipmStop = true;
    _lipmTrajReady = false;
  }

  _lipmCVar.notify_all();
  _lipmThread.join();  
}

void LipmWalkingCon::init(const RobotState &rs)
{
  if (_inited)
    return;
  //_curTrajIdx = 0;
  _mass = rs.m;
  rePlan(rs, true);
  
  _numStepsLeft = 0;
  _toeOffFlag = false;
  _inited = true; 
  _lipmStop = false;
  ikCon->setToRobotState(rs);
  _lipmThread = boost::thread(boost::bind(&LipmWalkingCon::genTrajWorker, this)); 

  printParams();
}

void LipmWalkingCon::genTrajWorker()
{
  Pose tmpLastFoot[LR];
  Pose com0;
  Eigen::Vector3d comd0;
  
  bool ret;
  size_t idx;

  // assuming we start really fast... 
  // and replan(rs, true) is called before this thread is spawned.
  assert(_refinedPlan.size() == _Nsteps);
  std::vector <SFootStep> localPlan = _refinedPlan;
  Traj<9,0> localFlTraj = _flTraj;
  Traj<9,0> localFrTraj = _frTraj;
  Traj<7,0> localRootTraj = _rootTraj;
  LipmVarHeightPlanner localLipm = _lipm;
  double iniTime = _initTime;
  double last_t0 = _refinedPlan[_refinedPlan.size()-1].time; 
  tmpLastFoot[LEFT] = _lastFootStep[LEFT];
  tmpLastFoot[RIGHT] = _lastFootStep[RIGHT];
  size_t nSteps = _Nsteps;

  int DSc_count = 0;

  printf("\n\n\n\n\nHello world from worker!!!\n\n\n\n\n");

  boost::mutex::scoped_lock lock(_lipmLock);
  while (1) 
  {   
    // if the main thread hasn't collected new trajectories,
    // i go to sleep
    while (_lipmTrajReady) {
      printf("\n\n\n\nworker goes to sleep\n\n\n\n");
      // atomically drops lock when go to sleep
      _lipmCVar.wait(lock);
    }
     
    printf("\n\n\n\nworker wakes up\n\n\n\n");
    
    // check stopping 
    if (_lipmStop) {
      break;
    }

    assert(localPlan.size() == _refinedPlan.size());

    //////////////////////////////////////////////////////////////
    // updates current foot step plan
    if (!_newPlanThreading.empty()) {
      for (size_t i = 0; i < _newPlanThreading.size(); i++) {
        last_t0 += lipm_step_duration;
        assert(_newPlanThreading[i].type != DSc);
        localPlan.push_back(SFootStep(_newPlanThreading[i].type, last_t0, _newPlanThreading[i].pose));
      }

      // wipe clean _newPlanThreading
      _newPlanThreading.clear();
    }

    //////////////////////////////////////////////////////////////
    // figure out the idx of the beginning of the next phase 
    idx = (size_t)((localPlan[0].time - iniTime) / __WALKCON_LIPM_TIME_STEP)-1;
    printf("_refinedPlan[0].time %g iniTime %g idx = %lu\n", localPlan[0].time, iniTime, idx);
    // time and planned com pos / vel at that time
    iniTime = localPlan[0].time;
    com0.pos = Eigen::Map<const Eigen::Vector3d> (localRootTraj[idx].pos);
    com0.rot = Eigen::Map<const Eigen::Quaterniond> (localRootTraj[idx].pos+3);
    comd0 = Eigen::Map<const Eigen::Vector3d> (localRootTraj[idx].vel);

    //////////////////////////////////////////////////////////////////////
    // update where the foot is 
    switch (localPlan[0].type) {
      case SSL:
        tmpLastFoot[RIGHT] = localPlan[0].pose;
        DSc_count = 0;
        _numStepsLeft--;
        break;

      case SSR:
        tmpLastFoot[LEFT] = localPlan[0].pose;
        DSc_count = 0;
        _numStepsLeft--;
        break;

      case DSc:
        DSc_count++;
        if (DSc_count > 2) {
          tmpLastFoot[LEFT] = _lastFootStep[LEFT];
          tmpLastFoot[RIGHT] = _lastFootStep[RIGHT];
        }
        break;
    }

    // remove the head of foot step plan
    localPlan.erase(localPlan.begin());

    // pad with dummy steps (standing still)
    last_t0 = localPlan[localPlan.size()-1].time;
    while(localPlan.size() < _Nsteps) {
      last_t0 += lipm_step_duration;
      localPlan.push_back(SFootStep(DSc, last_t0));
    }

    printf("worker replanning at %g, plan len %lu\n", iniTime, localPlan.size());
    assert(localPlan.size() >= nSteps);
    dumpPlan(localPlan); 
    //////////////////////////////////////////////////////////////////////

    lock.unlock();
    
    // generate new com and foot trajectores given a starting time, 
    // com pos / vel, and foot steps. this takes a while, and 
    // working only on local copies, should be thread safe
    ret = generateTrajThreading(localPlan, iniTime, com0, comd0, tmpLastFoot, nSteps, localFlTraj, localFrTraj, localRootTraj, localLipm);
    printf("generateTrajThreading returned %d\n", ret);

    assert(localFlTraj.size() > 1);
    assert(localFlTraj.size() == localFrTraj.size() && 
           localFrTraj.size() == localRootTraj.size());

    lock.lock();

    // save the results
    _lipmRetCode = ret;
    _lipmTrajReady = true;
    _refinedPlanThreading = localPlan;

    _flTrajThreading = localFlTraj;
    _frTrajThreading = localFrTraj;
    _rootTrajThreading = localRootTraj;
    _lipmThreading = localLipm;

    printf("worker size %lu %lu %lu\n", _flTrajThreading.size(), _frTrajThreading.size(), _rootTrajThreading.size());
  }
  
  printf("\n\n\n\n\nWorker exited!!!\n\n\n\n\n");
}

void LipmWalkingCon::set_footpos_knots(double t0, double t1, const double p0[3], const double p1[3], double z, double s, knot_t *k0, knot_t *k1, knot_t *k_mid)
{
  for (int i = 0; i < 3; i++) {
    k0[i].time = t0;
    k0[i].pos = p0[i];
    k0[i].vel = 0;
    k0[i].acc = 0;

    k1[i].time = t1;
    k1[i].pos = p1[i];
    k1[i].vel = 0;
    k1[i].acc = 0;

    k_mid[i].time = (t1+t0)/2.;
    k_mid[i].pos = 0;
    k_mid[i].vel = 0;
    k_mid[i].acc = 0;
  }
  for (int i = 0; i < 2; i++) {
    k_mid[i].pos = (1-s)*p0[i]+s*p1[i];
    k_mid[i].vel = (p1[i] - k_mid[i].pos) / (t1 - k_mid[i].time);
    //quintic_spline(k_mid[i].time, k0+i, k1+i, NULL, &(k_mid[i].vel), NULL);
  }
  k_mid[ZZ].pos = z;
}

void LipmWalkingCon::lookup_footpos(double t, const knot_t *k0, const knot_t *k1, const knot_t *k_mid, 
  double *pos, double *vel, double *acc)
{
  for (int i = 0; i < 3; i++) {
    if (t <= k_mid->time)
      quintic_spline(t, k0+i, k_mid+i, pos+i, vel+i, acc+i);
    else
      quintic_spline(t, k_mid+i, k1+i, pos+i, vel+i, acc+i);    
  }
}

bool LipmWalkingCon::generateTrajThreading(
  const std::vector <SFootStep> &localPlan, 
  double iniTime, 
  const Pose &com0, const Eigen::Vector3d &comd0, 
  const Pose p0[LR], size_t N_steps,
  Traj<9,0> &localFlTraj,
  Traj<9,0> &localFrTraj,
  Traj<7,0> &localRootTraj,
  LipmVarHeightPlanner &localLipm)
{
  // quintic spline knots for feet and yaw trajectory
  knot_t k0_l[3];
  knot_t k1_l[3];
  knot_t k_mid_l[3];
  //knot_t kz_mid_l;

  knot_t k0_r[3];
  knot_t k1_r[3];
  knot_t k_mid_r[3];
  //knot_t kz_mid_r;

  int state = DSc;
  int swingFoot = -1;
  int stanceFoot = -1;
  
  // desired zmp trajectory, used in lipm stuff to generate 
  // com trajectory
  Traj<3,3> zmp_d;

  double zmp_x[3] = {0};
  double zmp_u[3] = {0};
 
  //////////////////////////////////////////////////////////////
  // local copies
  localRootTraj.reset();
  localFlTraj.reset();
  localFrTraj.reset();
  std::vector <double> z_offset; // z offset
  
  double tmp[9] = {0};
  double tmpv[9] = {0};
  double tmpacc[9] = {0};

  // foot pose at the start / end of each step
  Eigen::Quaterniond root_q0;
  Eigen::Quaterniond root_q1;
  Pose foot_pos0[LR], foot_pos1[LR];
  Eigen::AngleAxisd wQ[LR];
  Eigen::Vector3d w[LR];
  Eigen::AngleAxisd root_wQ;
  Eigen::Vector3d root_w;
  
  // controls highest swing foot z
  double footz[LR] = {0};
  // scales mid point, 0 = pos0, 1 = pos1
  double s[LR] = {lipm_flat_swing_scale, lipm_flat_swing_scale};  

  assert(N_steps <= localPlan.size());

  // zmp stuff
  // desired zmp in world frame 
  double zmp0[2] = {0};
  double zmp1[2] = {0};
  // local zmp in foot frame
  double rel_zmp[LR][2] = {{0}, {0}};
  // desired com height
  double z0_d = lipm_z_d_normal;
  double z1_d = lipm_z_d_normal;
  double tmpx, tmpy, tmpt;
  double minz0, minz1;

  // time related
  double time = 0;
  double t0, t1_root, t1_foot;
  double dt = __WALKCON_LIPM_TIME_STEP;
  double time0 = iniTime;

  // init initial foot step to where I am now
  for (int i = 0; i < LR; i++)
    foot_pos1[i] = p0[i];
  root_q1 = com0.rot;
  
  // flag for double stance transition period only 
  // between SSL and SSR
  bool dsFlag = false;
  int trajState = DSc;
  
  // going Up or Down
  int mode = Flat;

  // for the next N steps
  for (size_t n = 0; n < N_steps; n++) 
  {
    // figure out swing foot, and SSL/SSR 
    state = localPlan[n].type;
    dsFlag = false;
    switch (state) {
      case SSL:
        swingFoot = RIGHT;
        stanceFoot = LEFT;
        dsFlag = true;
        break;
      case SSR:
        swingFoot = LEFT;
        stanceFoot = RIGHT;
        dsFlag = true;
        break;
      case DSc:
        dsFlag = false;
        break;
      default:
        assert(0);
    }

    // save foot locations at the beginning
    foot_pos0[LEFT] = foot_pos1[LEFT];
    foot_pos0[RIGHT] = foot_pos1[RIGHT];
    s[LEFT] = s[RIGHT] = lipm_flat_swing_scale;
    root_q0 = root_q1;

    if (state == DSc) {
      // rotate desired zmp based on current foot orientation
      Eigen::Vector3d v(lipm_zmp_x_dsc, 0, 0);
      //Eigen::Quaterniond gnd_q(Eigen::AngleAxisd((foot_pos0[LEFT][5]+foot_pos0[RIGHT][5])/2., Eigen::Vector3d::UnitZ()));
      Eigen::Quaterniond gnd_q = foot_pos0[LEFT].rot.slerp(0.5, foot_pos0[RIGHT].rot).normalized();
      Eigen::Vector3d offset = gnd_q.toRotationMatrix() * v; 

      // set desired zmp in world frame
      zmp1[XX] = (foot_pos0[LEFT].pos[XX]+foot_pos0[RIGHT].pos[XX])/2. + offset[XX];
      zmp1[YY] = (foot_pos0[LEFT].pos[YY]+foot_pos0[RIGHT].pos[YY])/2. + offset[YY];
      zmp0[XX] = zmp1[XX];
      zmp0[YY] = zmp1[YY];
      footz[LEFT] = foot_pos0[LEFT].pos[ZZ];
      footz[RIGHT] = foot_pos0[RIGHT].pos[ZZ];  
      
      // figure out desired com z at beginning and end of this step
      minz0 = std::min(foot_pos0[LEFT].pos[ZZ], foot_pos0[RIGHT].pos[ZZ]);
      minz1 = std::min(foot_pos1[LEFT].pos[ZZ], foot_pos1[RIGHT].pos[ZZ]);
      z0_d = minz0 + lipm_z_d_normal;
      z1_d = minz1 + lipm_z_d_normal;

      mode = Flat;
      
      // average 2 foot yaw
      Eigen::Vector3d zyx_l = quat2zyx(foot_pos1[LEFT].rot);
      Eigen::Vector3d zyx_r = quat2zyx(foot_pos1[RIGHT].rot);
      root_q1 = zyx2quat(Eigen::Vector3d(0, 0, avgAngle(zyx_l.z(),zyx_r.z())));
      /*
      root_q1 = avgQuatYaw(foot_pos1[LEFT].rot, foot_pos1[RIGHT].rot);
      */

      //root_q1 = foot_pos1[LEFT].rot.slerp(0.5, foot_pos1[RIGHT].rot).normalized();
    }
    else {
      // swing
      foot_pos1[stanceFoot] = foot_pos0[stanceFoot];
      foot_pos1[swingFoot] = localPlan[n].pose;

      if (foot_pos1[swingFoot].pos[ZZ] > foot_pos0[swingFoot].pos[ZZ])
        footz[swingFoot] = lipm_step_height + foot_pos1[swingFoot].pos[ZZ];
      else
        footz[swingFoot] = lipm_step_height + foot_pos0[swingFoot].pos[ZZ];
      footz[stanceFoot] = foot_pos1[stanceFoot].pos[ZZ]; 
       
      // figure out desired com z at beginning and end of this step
      minz0 = std::min(foot_pos0[LEFT].pos[ZZ], foot_pos0[RIGHT].pos[ZZ]);
      minz1 = std::min(foot_pos1[LEFT].pos[ZZ], foot_pos1[RIGHT].pos[ZZ]);
      
      // follow the swing foot yaw
      /*
      Eigen::Vector3d tmpzyx;
      root_q1 = avgQuatYaw(foot_pos1[LEFT].rot, foot_pos1[RIGHT].rot);
      */
      
      Eigen::Vector3d zyx_l = quat2zyx(foot_pos1[LEFT].rot);
      Eigen::Vector3d zyx_r = quat2zyx(foot_pos1[RIGHT].rot);
      Eigen::Vector3d tmpzyx = Eigen::Vector3d(0, 0, avgAngle(zyx_l.z(),zyx_r.z()));
      root_q1 = zyx2quat(tmpzyx);
      // next desired zmp is at the center of next foot hold
      Eigen::Vector3d v(Eigen::Vector3d::Zero());
      
      Eigen::Vector3d diff = foot_pos1[swingFoot].pos - foot_pos0[swingFoot].pos;
      printf("swing f0 %g %g %g\n", foot_pos0[swingFoot].pos[XX], foot_pos0[swingFoot].pos[YY], foot_pos0[swingFoot].pos[ZZ]);
      printf("swing f1 %g %g %g\n", foot_pos1[swingFoot].pos[XX], foot_pos1[swingFoot].pos[YY], foot_pos1[swingFoot].pos[ZZ]);
      double slope = diff[ZZ] / sqrt((diff[XX]*diff[XX]+diff[YY]*diff[YY]));
      if (sqrt((diff[XX]*diff[XX]+diff[YY]*diff[YY])) < 0.01)
        slope = 0;

      // going down
      if (slope <= lipm_down_thres) {
        mode = DownBlock;
        v[XX] = lipm_zmp_x_down;
        z0_d = minz0 + lipm_z_d_down;
        z1_d = minz1 + lipm_z_d_down;
        
        /*
        tmpzyx = quat2zyx(foot_pos1[swingFoot].rot);
        tmpzyx[YY] = -slope;
        foot_pos1[swingFoot].rot = zyx2quat(tmpzyx);
        */

        s[swingFoot] = lipm_down_swing_scale;
      }
      else if (slope >= lipm_up_thres) {
        mode = UpBlock;
        v[XX] = lipm_zmp_x_up;
        root_q1 = zyx2quat(Eigen::Vector3d(0, lipm_up_torso_pitch, tmpzyx.z()));
        if (root_q1.dot(root_q0) < 0)
          neg_quat(root_q1);
        z0_d = minz0 + lipm_z_d_up;
        z1_d = minz1 + lipm_z_d_up;

        /*
        tmpzyx = quat2zyx(foot_pos1[swingFoot].rot);
        tmpzyx[YY] = -slope;
        foot_pos1[swingFoot].rot = zyx2quat(tmpzyx);
        */

        s[swingFoot] = lipm_up_swing_scale;
      }
      else {
        mode = Flat;
        v[XX] = lipm_zmp_x_normal;
        //v[XX] = -0.08;
        tmpzyx = quat2zyx(foot_pos1[swingFoot].rot);
        tmpzyx[YY] = 0;//-0.05;
        foot_pos1[swingFoot].rot = zyx2quat(tmpzyx);  
        z0_d = minz0 + lipm_z_d_normal;
        z1_d = minz1 + lipm_z_d_normal;
        
        s[swingFoot] = lipm_flat_swing_scale;
      }

      //Eigen::Quaterniond gnd_q(Eigen::AngleAxisd(foot_pos0[stanceFoot][5], Eigen::Vector3d::UnitZ()));
      Eigen::Quaterniond gnd_q = foot_pos0[stanceFoot].rot;
      Eigen::Vector3d offset = gnd_q.toRotationMatrix() * v; 

      zmp1[XX] = foot_pos0[stanceFoot].pos[XX] + offset[XX];
      zmp1[YY] = foot_pos0[stanceFoot].pos[YY] + offset[YY];
      zmp0[XX] = zmp1[XX];
      zmp0[YY] = zmp1[YY];
    }

    // start and end time for this step
    t0 = time0;
    t1_foot = t1_root = localPlan[n].time;
    
    // minus ds period if we want double support transition
    if (dsFlag)
      t1_foot -= lipm_step_ds_duration;

    time0 = t1_root;

    printf("mode %s, state %d, t0 %g t1 %g, zmpx %g zmpy %g z0 %g z1 %g\n", TerrainModeNames[mode].c_str(), state, t0, t1_root, zmp1[XX], zmp1[YY], z0_d, z1_d);
    assert(t1_foot > t0);
 
    // make sure slerp goes the "short" way
    if (root_q1.dot(root_q0) < 0)
      neg_quat(root_q1);
    for (int i = 0; i < LR; i++) {
      if (foot_pos0[i].rot.dot(foot_pos1[i].rot) < 0)
        neg_quat(foot_pos1[i].rot);
    }

    // foot knot points
    //set_foot_knots_pos_only(t0, t1_foot, foot_pos0[LEFT].pos.data(), foot_pos1[LEFT].pos.data(), footz[LEFT], k0_l, k1_l, &kz_mid_l);
    //set_foot_knots_pos_only(t0, t1_foot, foot_pos0[RIGHT].pos.data(), foot_pos1[RIGHT].pos.data(), footz[RIGHT], k0_r, k1_r, &kz_mid_r);
    set_footpos_knots(t0, t1_foot, foot_pos0[LEFT].pos.data(), foot_pos1[LEFT].pos.data(), footz[LEFT], s[LEFT], k0_l, k1_l, k_mid_l);
    set_footpos_knots(t0, t1_foot, foot_pos0[RIGHT].pos.data(), foot_pos1[RIGHT].pos.data(), footz[RIGHT], s[RIGHT], k0_r, k1_r, k_mid_r);

    for (int i = 0; i < LR; i++) {
      wQ[i] = Eigen::AngleAxisd(foot_pos1[i].rot * (foot_pos0[i].rot.inverse()));
      w[i] = wQ[i].axis()*(wQ[i].angle()/(t1_foot-t0));
    }
    root_wQ = Eigen::AngleAxisd(root_q1 * (root_q0.inverse()));
    root_w = root_wQ.axis()*(root_wQ.angle()/(t1_foot-t0)); 

    //////////////////////////////////////////////////////////////////////////
    // generating trajectories with quintic spline
    // t1-dt/2. for double comparison edge case..
    for (time = t0; time < t1_root-dt/2.; time += dt) {
      trajState = state;

      // desired zmp in foot frame, from toe
      if (state == DSc) {
        rel_zmp[LEFT][XX] = lipm_zmp_x_dsc;
        rel_zmp[RIGHT][XX] = lipm_zmp_x_dsc;
      }
      else {
        if (mode == DownBlock) {
          rel_zmp[LEFT][XX] = lipm_zmp_x_down;
          rel_zmp[RIGHT][XX] = lipm_zmp_x_down;
        }
        else if (mode == UpBlock) {
          rel_zmp[LEFT][XX] = lipm_zmp_x_up;
          rel_zmp[RIGHT][XX] = lipm_zmp_x_up;
        } 
        else {
          rel_zmp[LEFT][XX] = lipm_zmp_x_normal;
          rel_zmp[RIGHT][XX] = lipm_zmp_x_normal;
        }
      }

      if (dsFlag && time >= t1_foot-dt/2.) {
        // transit into ssr
        if (state == SSL) {
          trajState = DSr;
          rel_zmp[LEFT][XX] = 0.26; //lipm_zmp_x_normal;
          //rel_zmp[LEFT][XX] = 0;
        }
        else if (state == SSR) {
          trajState = DSl;
          rel_zmp[RIGHT][XX] = 0.26; //lipm_zmp_x_normal;
          //rel_zmp[RIGHT][XX] = 0;
        }
        else
          assert(0);
      }

      // linear interp between zmp0 and zmp1 to get 
      // desired zmp / foot location, in world frame
      tmpt = (time-t0)/(t1_root-t0);
      tmpx = (1-tmpt)*zmp0[XX] + tmpt*zmp1[XX];
      tmpy = (1-tmpt)*zmp0[YY] + tmpt*zmp1[YY];
      
      // look up desired root yaw, xyz are bogus, will fix later
      //quintic_spline(time, &k0_yaw, &k1_yaw, root_pos+5, NULL, NULL);
      
      // slerp time
      tmpt = (time-t0)/(t1_foot-t0);
      tmpt = std::min(tmpt, 1.);
      tmpt = std::max(tmpt, 0.);

      for (int i = 0; i < 9; i++)
        tmp[i] = tmpv[i] = tmpacc[i] = 0;
      Eigen::Map <Eigen::Quaterniond> tmpQ(tmp+3);
      Eigen::Map <Eigen::Vector3d> tmpW(tmpv+3);
      tmpQ = root_q0.slerp(tmpt, root_q1).normalized();
      tmpW = root_w;
      localRootTraj.append(time, trajState, tmp, NULL, NULL, NULL);

      // build initial com zmp trajectory
      zmp_x[0] = tmpx;
      zmp_x[1] = tmpy;
      zmp_x[2] = std::max(z0_d, z1_d);

      zmp_u[0] = tmpx;
      zmp_u[1] = tmpy;
      zmp_u[2] = _mass * GRAVITY;
      zmp_d.append(time, trajState, zmp_x, NULL, NULL, zmp_u);
  
      if (state == DSc)
        z_offset.push_back((footz[LEFT] + footz[RIGHT]) / 2.);
      else
        z_offset.push_back(footz[stanceFoot]); 

      // slerp time
      tmpt = (time-t0)/(t1_foot-t0);
      tmpt = std::min(tmpt, 1.);
      tmpt = std::max(tmpt, 0.);
      
      for (int i = 0; i < 9; i++)
        tmp[i] = tmpv[i] = tmpacc[i] = 0;
      // save desired foot trajectory
      lookup_footpos(time, k0_l, k1_l, k_mid_l, tmp, tmpv, tmpacc);
      //lookup_foot_pos_only(time, k0_l, k1_l, &kz_mid_l, tmp, tmpv, tmpacc);
      // slerp between start and end
      tmpQ = foot_pos0[LEFT].rot.slerp(tmpt, foot_pos1[LEFT].rot).normalized();
      tmpW = w[LEFT];
      tmp[7] = rel_zmp[LEFT][XX];
      tmp[8] = rel_zmp[LEFT][YY];
      localFlTraj.append(time, mode, tmp, tmpv, tmpacc, NULL);

      for (int i = 0; i < 9; i++)
        tmp[i] = tmpv[i] = tmpacc[i] = 0;
      lookup_footpos(time, k0_r, k1_r, k_mid_r, tmp, tmpv, tmpacc);
      //lookup_foot_pos_only(time, k0_r, k1_r, &kz_mid_r, tmp, tmpv, tmpacc);
      // slerp between start and end
      tmpQ = foot_pos0[RIGHT].rot.slerp(tmpt, foot_pos1[RIGHT].rot).normalized();
      tmpW = w[RIGHT];
      tmp[7] = rel_zmp[RIGHT][XX];
      tmp[8] = rel_zmp[RIGHT][YY];
      localFrTraj.append(time, mode, tmp, tmpv, tmpacc, NULL);
    }
  }
  
  // call lipm to generate COM xyz trajectory
  Traj<3,3> com;
  double cur_rootx[6] = {com0.pos[XX], com0.pos[YY], com0.pos[ZZ], comd0[XX], comd0[YY], comd0[ZZ]};
  
  //LipmVarHeightPlanner lipm(_mass, dt, 1e-4);
  localLipm = LipmVarHeightPlanner(_mass, dt, lipm_Q, lipm_R);
  localLipm.getCOMTraj(zmp_d, cur_rootx, z_offset, com);
  
  //LipmVarHeightDDP lipm1(rs.m, dt);
  //lipm1.getCOMTraj(zmp_d, cur_rootx, com);

  //zmp_d.toFile("tmp/zmp_d", true, true);
  //com.toFile("tmp/com", true, true);

  assert(com.size() == localFlTraj.size() && 
         com.size() == localFrTraj.size() &&
         com.size() == localRootTraj.size());

  for (size_t t = 0; t < com.size(); t++) {
    for (int i = 0; i < 3; i++) {
      localRootTraj[t].pos[i] = com[t].pos[i];
      localRootTraj[t].vel[i] = com[t].vel[i];
      localRootTraj[t].acc[i] = com[t].acc[i];
    }
  }
  
  return true;
}

void LipmWalkingCon::dumpPlan(const std::vector <SFootStep> &plan) const
{
  for (size_t i = 0; i < plan.size(); i++) 
    printf("fs plan time %3g, type %1d, x %5g y %5g z %5g qx %5g qy %5g qz %5g qw %5g\n", 
      plan[i].time, plan[i].type, plan[i].pose.pos[0], plan[i].pose.pos[1],  plan[i].pose.pos[2], plan[i].pose.rot.x(), plan[i].pose.rot.y(), plan[i].pose.rot.z(), plan[i].pose.rot.w()); 
}

bool LipmWalkingCon::refineFootSteps(const RobotState &rs, const std::vector <SFootStep> &fsPlan)
{
  boost::mutex::scoped_lock lock(_lipmLock);
  SFootStep tmp;
  Eigen::Vector3d zyx;
 
  for (size_t i = 0; i < fsPlan.size(); i++) {
    if (fsPlan[i].type == DSc) 
      continue;
    _newPlanThreading.push_back(fsPlan[i]);
    _numStepsLeft++;
  }

  return true;
}

bool LipmWalkingCon::updateFootSteps(const RobotState &rs, const std::vector <SFootStep> &fsPlan)
{
  if (!_inited)
    init(rs);

  //_originalPlan = fsPlan;
  return refineFootSteps(rs, fsPlan);
}

// whenever this is called, worker thread should be asleep!!
bool LipmWalkingCon::rePlan(const RobotState &rs, bool firstTime) 
{
  double last_t0;
  bool ret;
 
  printf("\t\t rePlan called at %g %d\n", rs.time, firstTime);

  boost::mutex::scoped_lock lock(_lipmLock);

  //////////////////////////////////////////////////////////////////
  // do the first 3 steps my self 
  if (firstTime) {
    _refinedPlan.clear();
    for (int i = 0; i < LR; i++) {
      _lastFootStep[i].pos = Eigen::Map <const Eigen::Vector3d> (rs.feet[i].w_mid_pos);
      _lastFootStep[i].rot = onlyYaw(rs.feet[i].w_q);
    }

    last_t0 = rs.time;
    _initTime = rs.time;
    // pad with dummy steps
    while(_refinedPlan.size() < _Nsteps) {
      last_t0 += lipm_step_duration;
      _refinedPlan.push_back(SFootStep(DSc, last_t0));
    }
    printf("planning first time at %g, plan len %lu\n", rs.time, _refinedPlan.size());
    dumpPlan(_refinedPlan);
    
    printf("rs xyz %g %g %g\n", rs.com[0], rs.com[1], rs.com[2]);
    Pose com0;
    com0.pos = Eigen::Map <const Eigen::Vector3d> (rs.com);
    com0.rot = rs.rootq;
    ret = generateTrajThreading(_refinedPlan, rs.time, 
      com0, Eigen::Map <const Eigen::Vector3d> (rs.comd), 
      _lastFootStep, _Nsteps, _flTraj, _frTraj, _rootTraj, _lipm);

    // signals worker thread to plan for the next 3 steps
    _lipmTrajReady = false;

    //getchar();
  }
  //////////////////////////////////////////////////////////////////
  else {
    //assert(_lipmTrajReady);
    if (!_lipmTrajReady)
      return true;

    _refinedPlan = _refinedPlanThreading;

    ret = _lipmRetCode;
    _flTraj = _flTrajThreading;
    _frTraj = _frTrajThreading;
    _rootTraj = _rootTrajThreading;
    _lipm = _lipmThreading;
    _lipmTrajReady = false;
    
    ///////////////////////////////////////////////////////////////////
    // should only be used when in DSC for a long time!
    _lastFootStep[LEFT] = Pose(Eigen::Map<const Eigen::Vector3d>(rs.feet[LEFT].w_mid_pos), rs.feet[LEFT].w_q);
    _lastFootStep[RIGHT] = Pose(Eigen::Map<const Eigen::Vector3d>(rs.feet[RIGHT].w_mid_pos), rs.feet[RIGHT].w_q);
    ///////////////////////////////////////////////////////////////////

    assert(_flTraj.size() == _frTraj.size() &&
           _frTraj.size() == _rootTraj.size() &&
           _rootTraj.size() > 1);
  }

  // wakes up worker
  lock.unlock();
  _lipmCVar.notify_all();
 
  //getchar();
  return ret;
}

int LipmWalkingCon::control(RobotState &rs, Command &cmd)
{
  if (!_inited)
    init(rs);

  if (rs.time >= _refinedPlan[0].time-rs.timeStep/2.) {
    rePlan(rs, false);
  }

  size_t idx = _rootTraj.getIdx(rs.time);
  assert(_rootTraj.size() == _flTraj.size() && 
         _rootTraj.size() == _frTraj.size() && 
         _refinedPlan.size() >= _Nsteps &&
         idx < _rootTraj.size());

  int mode = _flTraj[idx].type;
  _toeOffFlag = false;

  fillDesireds(rs, _rootTraj[idx], _flTraj[idx], _frTraj[idx]);

  //dvec_copy(cmd.joints_d, ikcmd.joints, N_JOINTS);

  if (!idCon->ID(rs, idcmd, cmd))
    return -1;

  //if (!ikCon->IK(ikcmd, cmd))
  //  return -1;
 
  return 0;
}

void LipmWalkingCon::fillDesireds(const RobotState &rs, const TrajPoint<7,0> &root, const TrajPoint<9,0> &fl, const TrajPoint<9,0> &fr)
{
  // set up desired joint pos, vel and acc
  for (int i = 0; i < N_JOINTS; i++) {
    if (i >= A_L_LEG_UHZ && i <= A_R_LEG_LAX) { 
      ikcmd.joints[i] = INFINITY;
      ikcmd.jointsd[i] = INFINITY;
      idcmd.jointsdd[i] = INFINITY;
    }
    else if ((i >= A_BACK_LBZ && i <= A_BACK_UBX) && (rs.getType() != RobotState::TYPE_HAT_NS)) {
      ikcmd.joints[i] = INFINITY;
      ikcmd.jointsd[i] = INFINITY;
      idcmd.jointsdd[i] = INFINITY;
    }
    else {
      ikcmd.joints[i] = Command::BDI_joints[i];

      if (i == A_NECK_AY)
        ikcmd.joints[i] = _neck_ay_d;

      // FOR IK
      ikcmd.jointsd[i] = 0;
      //cmd.jointsd[i] = ik_joint_p*(cmd.joints[i] - rs.joints[i]);
      idcmd.jointsdd[i] = 0 
        + fd_joint_p*(ikcmd.joints[i]-rs.joints[i]) 
        + fd_joint_d*(ikcmd.jointsd[i]-rs.jointsd[i]); 
    }
  }
  
  int contactState = root.type;
  int mode = fl.type;
  
   
  
  //////////////////////////////////////////////////////////////////////////////////////////
  // root 
  Eigen::Vector3d com_acc = _lipm.getCOMAcc(_rootTraj.getIdx(rs.time), Eigen::Map<const Eigen::Vector3d> (rs.com), Eigen::Map<const Eigen::Vector3d> (rs.comd));
  
  for (int i = 0; i < 3; i++) {
    ikcmd.com[i] = root.pos[i];
    ikcmd.comd[i] = root.vel[i];
    idcmd.comdd[i] = com_acc[i];
    idcmd.comdd[i] = std::min(idcmd.comdd[i], 10.);
    idcmd.comdd[i] = std::max(idcmd.comdd[i], -10.);
    //idcmd.comdd[i] = root.acc[i] 
    //  + fd_com_p[i]*(ikcmd.com[i]-rs.com[i]) 
    //  + fd_com_d[i]*(ikcmd.comd[i]-rs.comd[i]);

    ikcmd.root[i] = INFINITY;
    ikcmd.rootd[i] = INFINITY;
    idcmd.rootdd[i] = INFINITY;
  }
   
  // orientation tracking
  ikcmd.torsoQ = Eigen::Map<const Eigen::Quaterniond> (root.pos+3);

  if (ikcmd.torsoQ.dot(rs.utorsoq) < 0)
    neg_quat(ikcmd.torsoQ);
  
  Eigen::Vector3d Rotvec = quatMinus(rs.utorsoq, ikcmd.torsoQ);

  for (int i = 0; i < 3; i++) {
    // FOR IK
    ikcmd.torsoW[i] = root.vel[i+3];
    //ikcmd.torsoW[i] = root.vel[i+3] - ik_com_p[i+3]*(Rotvec[i]);
    idcmd.torsoWd[i] = 0 
      - fd_com_p[i+3]*(Rotvec[i]) 
      + fd_com_d[i+3]*(ikcmd.torsoW[i]-rs.utorsow[i]);
  }

  ikcmd.pelvQ = Eigen::Map<const Eigen::Quaterniond> (root.pos+3);
  if (ikcmd.pelvQ.dot(rs.rootq) < 0)
    neg_quat(ikcmd.pelvQ);
  
  Rotvec = quatMinus(rs.rootq, ikcmd.pelvQ);

  for (int i = 0; i < 3; i++) {
    ikcmd.pelvW[i] = root.vel[i+3];
    idcmd.pelvWd[i] = 0 
      - fd_com_p[i+3]*(Rotvec[i]) 
      + fd_com_d[i+3]*(ikcmd.pelvW[i]-rs.rootd[i+3]);
  }

  /*
  // desired orientation in body frame
  Eigen::Quaterniond world_to_body = rs.rootq.inverse().normalized();
  Eigen::Quaterniond body_q_d = world_to_body * ikcmd.pelvQ;
  if (body_q_d.dot(Eigen::Quaterniond::Identity()) < 0) 
    neg_quat(body_q_d);
  
  Rotvec = quatMinus(Eigen::Quaterniond::Identity(), body_q_d);

  Eigen::Vector3d body_w_d = world_to_body.toRotationMatrix() * Eigen::Map<const Eigen::Vector3d> (root.vel+3);
  
  for (int i = 0; i < 3; i++) {
    // FOR IK
    ikcmd.root_b_w[i] = body_w_d[i];
    //ikcmd.root_b_w[i] = body_w_d[i] - ik_com_p[i+3]*(Rotvec[i]);
    idcmd.root_b_wd[i] = 0
      - fd_com_p[i+3]*(Rotvec[i])
      + fd_com_d[i+3]*(ikcmd.root_b_w[i]-rs.root_b_w[i]);
  }
  // world frame angular velocity and acceleration
  dvec_copy(ikcmd.rootd+3, root.vel+3, 3);
  Eigen::Map <Eigen::Vector3d> root_w_wd_d(idcmd.pelvWd);
  Eigen::Map <Eigen::Vector3d> root_b_wd_d(idcmd.root_b_wd);
  root_w_wd_d = rs.rootq.toRotationMatrix() * root_b_wd_d;
  */

  /////////////////////////////////////////////////////////////
  // left foot position
  for (int j = 0; j < 3; j++) {
    ikcmd.foot[LEFT][j] = fl.pos[j];
    // swing
    if (rs.contactState != SSL && !isDS(rs.contactState)) {
      // FOR IK
      ikcmd.footd[LEFT][j] = fl.vel[j];
      //ikcmd.footd[LEFT][j] = fl.vel[j] + ik_foot_p[j]*(ikcmd.foot[LEFT][j]-rs.feet[LEFT].w_sensor_pos[j]);
      idcmd.footdd[LEFT][j] = fl.acc[j] 
        + fd_foot_p[j]*(ikcmd.foot[LEFT][j]-rs.feet[LEFT].w_mid_pos[j]) 
        + fd_foot_d[j]*(ikcmd.footd[LEFT][j]-rs.feet[LEFT].w_sensor_vel[j]);
    }
    // stance
    else {
      ikcmd.footd[LEFT][j] = 0;
      idcmd.footdd[LEFT][j] = 0;
    }
  }
  
  // left foot orientation
  ikcmd.footQ[LEFT] = Eigen::Map <Eigen::Quaterniond> (&(fl.pos[3]));
  if (ikcmd.footQ[LEFT].dot(rs.feet[LEFT].w_q) < 0) 
    neg_quat(ikcmd.footQ[LEFT]);
  
  Rotvec = quatMinus(rs.feet[LEFT].w_q, ikcmd.footQ[LEFT]);

  for (int i = 3; i < 6; i++) {
    // swing
    if (rs.contactState != SSL && !isDS(rs.contactState)) {
      // FOR IK
      ikcmd.footd[LEFT][i] = fl.vel[i];
      //ikcmd.footd[LEFT][i] = fl.vel[i] - ik_foot_p[i]*Rotvec[i-3];
      idcmd.footdd[LEFT][i] = 0 
        + fd_foot_d[i]*(ikcmd.footd[LEFT][i]-rs.feet[LEFT].w_sensor_vel[i]);
      if (rs.feet[LEFT].b_F[ZZ] < 100)
        idcmd.footdd[LEFT][i] -= fd_foot_p[i]*(Rotvec[i-3]);
    }                                      
    // stance
    else {
      ikcmd.footd[LEFT][i] = 0;
      idcmd.footdd[LEFT][i] = 0;
    }
  }
  if (rs.contactState == DSr && _toeOffFlag) {
    Eigen::Vector3d v(0, fd_toeoff_y, 0);
    Eigen::Map <Eigen::Vector3d> footdd_d(&(idcmd.footdd[LEFT][3]));
    footdd_d = ikcmd.footQ[LEFT].toRotationMatrix() * v;
  }
 
  /////////////////////////////////////////////////////////////
  // right foot position
  for (int j = 0; j < 3; j++) {
    ikcmd.foot[RIGHT][j] = fr.pos[j];
    // swing
    if (rs.contactState != SSR && !isDS(rs.contactState)) {
      // FOR IK
      ikcmd.footd[RIGHT][j] = fr.vel[j];
      //ikcmd.footd[RIGHT][j] = fr.vel[j] + ik_foot_p[j]*(ikcmd.foot[RIGHT][j]-rs.feet[RIGHT].w_sensor_pos[j]);
      idcmd.footdd[RIGHT][j] = fr.acc[j] 
        + fd_foot_p[j]*(ikcmd.foot[RIGHT][j]-rs.feet[RIGHT].w_mid_pos[j]) 
        + fd_foot_d[j]*(ikcmd.footd[RIGHT][j]-rs.feet[RIGHT].w_sensor_vel[j]);
    }
    // stance
    else {
      ikcmd.footd[RIGHT][j] = 0;
      idcmd.footdd[RIGHT][j] = 0;
    }
  }
   
  // right foot position
  ikcmd.footQ[RIGHT] = Eigen::Map <Eigen::Quaterniond> (&(fr.pos[3]));
  if (ikcmd.footQ[RIGHT].dot(rs.feet[RIGHT].w_q) < 0) 
    neg_quat(ikcmd.footQ[RIGHT]);
  
  Rotvec = quatMinus(rs.feet[RIGHT].w_q, ikcmd.footQ[RIGHT]);

  for (int i = 3; i < 6; i++) {
    // swing
    if (rs.contactState != SSR && !isDS(rs.contactState)) {
      // FOR IK
      ikcmd.footd[RIGHT][i] = fl.vel[i];
      //ikcmd.footd[RIGHT][i] = fl.vel[i] - ik_foot_p[i]*Rotvec[i-3];
      idcmd.footdd[RIGHT][i] = 0 
        + fd_foot_d[i]*(ikcmd.footd[RIGHT][i]-rs.feet[RIGHT].w_sensor_vel[i]);
      if (rs.feet[RIGHT].b_F[ZZ] < 100)
        idcmd.footdd[RIGHT][i] -= fd_foot_p[i]*(Rotvec[i-3]);
    }
    // stance
    else {
      ikcmd.footd[RIGHT][i] = 0;
      idcmd.footdd[RIGHT][i] = 0;
    }
  }
  if (rs.contactState == DSl && _toeOffFlag) {
    Eigen::Vector3d v(0, fd_toeoff_y, 0);
    Eigen::Map <Eigen::Vector3d> footdd_d(&(idcmd.footdd[RIGHT][3]));
    footdd_d = ikcmd.footQ[RIGHT].toRotationMatrix() * v;
  }

  
  double yaw[2], yaw_avg;
  Eigen::Vector3d zyx = quat2zyx(rs.feet[LEFT].w_q);
  yaw[LEFT] = zyx[2];
  zyx = quat2zyx(rs.feet[RIGHT].w_q);
  yaw[RIGHT] = zyx[2];
  
  // weight distribution
  switch (contactState) {
    case DSc:
      double dist[2];
      for (int side = 0; side < LR; side++) {
        for (int i = 0; i < 2; i++)
          dist[side] = sqrt((ikcmd.com[i]-ikcmd.foot[side][i])*(ikcmd.com[i]-ikcmd.foot[side][i]));
      }
      idcmd.wl = 1 - dist[LEFT] / (dist[LEFT]+dist[RIGHT]);
      idcmd.wl = (idcmd.wl > 0.9) ? 0.9 : idcmd.wl; 
      idcmd.wl = (idcmd.wl < 0.1) ? 0.1 : idcmd.wl;  
      yaw_avg = (yaw[LEFT] + yaw[RIGHT]) / 2.;
      //idcmd.wl = 0.5;
      break;
    case SSL:
      idcmd.wl = 1;
      yaw_avg = yaw[LEFT];
      break;
    case SSR:
      idcmd.wl = 0;
      yaw_avg = yaw[RIGHT];
      break;
    case DSl:
      idcmd.wl = 1;
      yaw_avg = yaw[LEFT];
      break;
    case DSr:
      idcmd.wl = 0;
      yaw_avg = yaw[RIGHT];
      break;
    default:
      assert(0);
  }
  /*
  // CoP
  idcmd.b_cop[LEFT][XX] = fl.pos[7];
  idcmd.b_cop[LEFT][YY] = fl.pos[8]; 
  idcmd.b_cop[RIGHT][XX] = fr.pos[7];
  idcmd.b_cop[RIGHT][YY] = fr.pos[8];
  */
  double z = (rs.feet[LEFT].w_sensor_pos[ZZ] + rs.feet[RIGHT].w_sensor_pos[ZZ]) / 2.;
  Eigen::Matrix2d rot;
  rot << cos(yaw_avg), -sin(yaw_avg), sin(yaw_avg), cos(yaw_avg);

  // fix cop stuff
  for (int i = 0; i < 2; i++)  
    idcmd.cop[i] = rs.com[i] - idcmd.comdd[i] * (rs.com[ZZ] - z) / GRAVITY;
 
  // doesn't actually in world frame, is the offset, but not orientation
  splitCoP(idcmd.cop, idcmd.wl, rs.feet[LEFT].w_sensor_pos, rs.feet[RIGHT].w_sensor_pos, idcmd.w_cop[LEFT], idcmd.w_cop[RIGHT]);

  for (int side = 0; side < LR; side++) {
    Eigen::Vector2d tmp(idcmd.w_cop[side]);
    zyx = quat2zyx(rs.feet[side].w_q);
    Eigen::Matrix2d world_to_foot;
    world_to_foot(0,0) = cos(-zyx[ZZ]);
    world_to_foot(0,1) = -sin(-zyx[ZZ]);
    world_to_foot(1,0) = sin(-zyx[ZZ]);
    world_to_foot(1,1) = cos(-zyx[ZZ]);
    Eigen::Map <Eigen::Vector2d> b_cop(idcmd.b_cop[side]);
    b_cop = world_to_foot * tmp;
    
    // add back the pos from foot
    idcmd.w_cop[side][XX] += rs.feet[side].w_sensor_pos[XX];
    idcmd.w_cop[side][YY] += rs.feet[side].w_sensor_pos[YY];
  }
}

