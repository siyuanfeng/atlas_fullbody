/*
 * =====================================================================================
 *
 *       Filename:  AtlasStaticWalkingCon.cpp
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  05/25/2013 11:00:46 AM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  YOUR NAME (), 
 *        Company:  
 *
 * =====================================================================================
 */

#include "AtlasStaticWalkingCon.h"
#include "Eigen_utils.hpp"
#include <map>
#include <boost/thread.hpp>

static double hack_cop(double kny);


AtlasStaticWalkingCon::AtlasStaticWalkingCon()
{
  buildLookup();

  com_z = 0.81;
  ss_height = 0.1;
  fd_joint_p = 100;
  fd_joint_d = 10;
  ss_lower_z_vel = -0.008;

  liftoff_height = 0.02;
  liftoff_rate = 0.03;

  
  for (int i = 0; i < 3; i++) {
    fd_com_p[i] = 100;
    fd_com_d[i] = 10;
    fd_com_p[i+3] = 150;
    fd_com_d[i+3] = 10;

    fd_foot_p[i] = 150;
    fd_foot_d[i] = 10;

    fd_foot_p[i+3] = 150;
    fd_foot_d[i+3] = 10;
  }

  com_int_i[LEFT] = com_int_i[RIGHT] = 0;
  foot_int_i[LEFT] = foot_int_i[RIGHT] = 0;

  touch_down_fz_thres = 0.02;

  _inited = false;

  _footSteps.clear();
}

AtlasStaticWalkingCon::~AtlasStaticWalkingCon()
{
  _footSteps.clear();
}

void AtlasStaticWalkingCon::init(const RobotState &rs)
{
  _stateTime = 0;
  _stepCount = 0;
  ikCon->setToRobotState(rs);
  gotoState(rs, Idle);
 
  // set up last time knots
  for (int i = 0; i < 3; i++)
    _com1[i].pos = rs.com[i];
  
  // hack pelvis z height
  _com1[ZZ].pos = rs.root[ZZ];
  _pelvQ1 = rs.rootq;
  _torsoQ1 = rs.utorsoq;
  _wl1.pos = 0.5;

  setupComDs(rs, -1, -1); 
  _inited = true;

  ikCon->setToRobotState(rs);

  _doToeOff = false;
  _neck_ay_d = 0;
  _lastFeet[LEFT] = Eigen::Map <const Eigen::Vector3d> (rs.feet[LEFT].w_sensor_pos); 
  _lastFeet[RIGHT] = Eigen::Map <const Eigen::Vector3d> (rs.feet[RIGHT].w_sensor_pos); 
  _swingLegJointMode = false;
  _com_d_over_foot[XX] = Foot::ankle_to_center_offset[XX];
  _com_d_over_foot[YY] = Foot::ankle_to_center_offset[YY];

  for (int side = 0; side < LR; side++) {
    _foot_d_i[side][XX] = 0;
    _foot_d_i[side][YY] = 0;
  }
  _com_int_flag[LEFT] = _com_int_flag[RIGHT] = false;
                               
  // bring arms down slowly
  for (int i = 0; i < N_JOINTS; i++) {
    if (i == A_L_ARM_ELX)
      set_single_knot(rs.time, rs.time+5, rs.joints[i], 1, _joints0_d+i, _joints1_d+i);
    else if (i == A_R_ARM_ELX)
      set_single_knot(rs.time, rs.time+5, rs.joints[i], -1, _joints0_d+i, _joints1_d+i);
    else
      set_single_knot(rs.time, rs.time+5, rs.joints[i], Command::BDI_joints[i], _joints0_d+i, _joints1_d+i);
  }
  
  for (int side = 0; side < LR; side++) {
    _com_d_i[side][0] = _com_d_i_ini[side][0];
    _com_d_i[side][1] = _com_d_i_ini[side][1];
  }
  
  printParams();
}

void AtlasStaticWalkingCon::gotoState(const RobotState &rs, AtlasStaticWalkingConState nxt)
{
  int old = _state;
  _state = nxt;
  _stateTime = 0;
  //_com_int_flag[LEFT] = _com_int_flag[RIGHT] = false;
  switch (nxt) {
    case Idle:
      _stepCount = 0;
      _contactState = DSc;
      break;
    case FixCopLeft:
      _contactState = DSc;
      break;
    case FixCopRight:
      _contactState = DSc;
      break;
    case ShiftComToRight:
    case ShiftComToLeft:
      _contactState = DSc;
      break;
    case LiftoffLeft:
    case SwingUpLeft:
    case SwingDownLeft:
      _contactState = SSR;
      break;
    case LiftoffRight:
    case SwingUpRight:
    case SwingDownRight:
      _contactState = SSL;
      break;
  }

  fprintf(stderr, "ASW \"%s\" -> \"%s\" @ %g\n", conModeNames[old].c_str(), conModeNames[_state].c_str(), rs.time);
  fprintf(stdout, "ASW \"%s\" -> \"%s\" @ %g\n", conModeNames[old].c_str(), conModeNames[_state].c_str(), rs.time);
}

int AtlasStaticWalkingCon::control(RobotState &rs, Command &cmd)
{
  _stateTime += rs.timeStep;

  double tmp[7] = {0}, tmpv[7] = {0}, tmpacc[7] = {0};
  Eigen::Map<Eigen::Vector3d> tmpPos(tmp);
  Eigen::Map<Eigen::Vector3d> tmpVel(tmpv);
  Eigen::Map<Eigen::Vector3d> tmpAcc(tmpacc);
  Eigen::Map<Eigen::Quaterniond> tmpQ(tmp+3);
  Eigen::Map<Eigen::Vector3d> tmpW(tmpv+3);
  Eigen::Map<Eigen::Vector3d> tmpWd(tmpacc+3);
  double t;
  double t1;
  Eigen::Map<const Eigen::Vector3d> rootd(rs.rootd);
  Eigen::Map<const Eigen::Vector3d> rootw(rs.rootd+3);
  Eigen::Vector3d zyx;

  TrajPoint <12,0> root_d;
  updateKnot(rs);
  interp(rs, root_d);

  TrajPoint <7,0> foot_d[LR];
  for (int side = 0; side < LR; side++) {
    dvec_copy(tmp, rs.feet[side].w_sensor_pos, 3);
    tmpQ = rs.feet[side].w_q;
    foot_d[side] = TrajPoint<7,0>(rs.time, rs.contactState, tmp, NULL, NULL, NULL);  
  }

  static int good_cop_ctr = 0;
  double lift_end_time = _com1[0].time;
  Eigen::Vector3d nxtFootLoc;

  // hack
  _footStage[LEFT] = _footStage[RIGHT] = WalkingCon::Planted;
  ikcmd.footRef[LEFT] = ikcmd.footRef[RIGHT] = Foot::RefSensor;
  idcmd.footRef[LEFT] = idcmd.footRef[RIGHT] = Foot::RefSensor;

  // hack toe off
  if (rs.joints[A_R_LEG_KNY] < 0.5 && !_doToeOff && _state == ShiftComToLeft) {
    _toeOffFoot = RIGHT;
    _doToeOff = true;
  }
  if (rs.joints[A_L_LEG_KNY] < 0.5 && !_doToeOff && _state == ShiftComToRight) {
    _toeOffFoot = LEFT;
    _doToeOff = true;
  }
  if (_doToeOff) {
    _footStage[_toeOffFoot] = WalkingCon::LiftOff;
    ikcmd.footRef[_toeOffFoot] = idcmd.footRef[_toeOffFoot] = Foot::RefToe;

    tmpQ = rs.feet[_toeOffFoot].w_q;
    tmpW.setZero();
    tmpWd.setZero();
    dvec_copy(tmp, rs.feet[_toeOffFoot].w_toe_pos, 3);
    tmpVel.setZero();
    tmpAcc.setZero();
    foot_d[_toeOffFoot] = TrajPoint<7,0>(rs.time, rs.contactState, tmp, tmpv, tmpacc, NULL);
  }

  switch (_state) {
    case Idle:
      fillDesireds(rs, root_d, foot_d);
     
      if (_stateTime >= ds_duration) {
        if (procNextStep(rs)) {
          if (_stanceFoot == LEFT) {
            setupComDs(rs, LEFT, LEFT); 
            gotoState(rs, ShiftComToLeft);
          }
          else if (_stanceFoot == RIGHT) {
            setupComDs(rs, LEFT, RIGHT); 
            gotoState(rs, ShiftComToRight);
          }
        } 
      }
      
      break;
  
    case FixCopLeft:
      fillDesireds(rs, root_d, foot_d);
      _footStage[RIGHT] = WalkingCon::LiftOff;

      if (fabs(w_cop2b_cop(rs,LEFT)[YY]) < 1e-2)
        good_cop_ctr++;
      else
        good_cop_ctr = 0;

      if (good_cop_ctr > 200) {
        ikCon->foot_q_weight = ikCon->IK_FOOT_WEIGHT / 1e1;
        good_cop_ctr = 0;
        setupSwingLiftoff(rs);
        setupComSs(rs);

        gotoState(rs, LiftoffRight); 
      }

      break;

    case FixCopRight:
      fillDesireds(rs, root_d, foot_d);
      _footStage[LEFT] = WalkingCon::LiftOff;
      
      if (fabs(w_cop2b_cop(rs,RIGHT)[YY]) < 1e-2)
        good_cop_ctr++;
      else
        good_cop_ctr = 0;

      if (good_cop_ctr > 200) {
        ikCon->foot_q_weight = ikCon->IK_FOOT_WEIGHT / 1e1;
        good_cop_ctr = 0;
        setupSwingLiftoff(rs);
        setupComSs(rs);

        gotoState(rs, LiftoffLeft);
      }

      break;

    case ShiftComToLeft:
      fillDesireds(rs, root_d, foot_d);
      
      if (_stateTime < 0.2 && _shiftCOMstart != -1)
        _footStage[LEFT] = WalkingCon::TouchDown;
      if (_stateTime > ds_duration - 0.2)
        _footStage[RIGHT] = WalkingCon::LiftOff;

      if (_stateTime >= ds_duration) {
        _com_int_flag[LEFT] = true;
        gotoState(rs, FixCopLeft);
      }

      break;

    case ShiftComToRight:
      fillDesireds(rs, root_d, foot_d);

      if (_stateTime < 0.2 && _shiftCOMstart != -1)
        _footStage[RIGHT] = WalkingCon::TouchDown;
      if (_stateTime > ds_duration - 0.2)
        _footStage[LEFT] = WalkingCon::LiftOff;

      if (_stateTime >= ds_duration) {
        _com_int_flag[RIGHT] = true;
        gotoState(rs, FixCopRight);
      }
      
      break;
    
    case LiftoffLeft:
      interpFoot(rs, foot_d[LEFT]);
      fillDesireds(rs, root_d, foot_d);
      _footStage[LEFT] = WalkingCon::LiftOff;

      if (_stateTime > _liftoffDuration) {
        _doToeOff = false;
        setupSwingUp(rs);
        gotoState(rs, SwingUpLeft);
      }
      
      break;
 
    case LiftoffRight:
      interpFoot(rs, foot_d[RIGHT]);
      fillDesireds(rs, root_d, foot_d);
      _footStage[RIGHT] = WalkingCon::LiftOff;

      if (_stateTime > _liftoffDuration) {
        _doToeOff = false;
        setupSwingUp(rs);
        gotoState(rs, SwingUpRight);
      }
      
      break;

    case SwingUpLeft:
      interpFoot(rs, foot_d[LEFT]);
      fillDesireds(rs, root_d, foot_d);
      _footStage[LEFT] = WalkingCon::Swing;
      
      if (rs.time > _swgFoot1[0].time) 
      {
        ikCon->foot_q_weight = ikCon->IK_FOOT_WEIGHT;
        setupSwingDown(rs);
        gotoState(rs, SwingDownLeft);
      } 

      break;

    case SwingUpRight:
      interpFoot(rs, foot_d[RIGHT]);
      fillDesireds(rs, root_d, foot_d);
      _footStage[RIGHT] = WalkingCon::Swing;
      
      if (rs.time > _swgFoot1[0].time) 
      {
        ikCon->foot_q_weight = ikCon->IK_FOOT_WEIGHT;
        setupSwingDown(rs);
        gotoState(rs, SwingDownRight);
      }  

      break;

    case SwingDownLeft:
      interpFoot(rs, foot_d[LEFT]);
      if (_stateTime > ss_duration - 0.5)
        _footStage[LEFT] = WalkingCon::TouchDown;
      else 
        _footStage[LEFT] = WalkingCon::Swing;
      
      fillDesireds(rs, root_d, foot_d);

      if (rs.feet[LEFT].b_F[ZZ] > touch_down_fz_thres)
      { 
        printf("left fz %g\n", rs.feet[LEFT].b_F[ZZ]);

        _lastFeet[LEFT] = Eigen::Map <const Eigen::Vector3d> (rs.feet[LEFT].w_sensor_pos); 

        _com_int_flag[RIGHT] = false;
        // has new steps
        if (procNextStep(rs)) {
          if (_stanceFoot == LEFT) {
            setupComDs(rs, LEFT, LEFT); 
            gotoState(rs, ShiftComToLeft);
          }
          else if (_stanceFoot == RIGHT) {
            setupComDs(rs, LEFT, RIGHT); 
            gotoState(rs, ShiftComToRight);
          }
        }
        else {
          gotoState(rs, Idle);
          setupComDs(rs, LEFT, -1); 
        }
      }
      break;

    case SwingDownRight:
      interpFoot(rs, foot_d[RIGHT]);
      if (_stateTime > ss_duration - 0.5)
        _footStage[RIGHT] = WalkingCon::TouchDown;
      else 
        _footStage[RIGHT] = WalkingCon::Swing; 
      
      fillDesireds(rs, root_d, foot_d);

      // check if foot not moving
      if (rs.feet[RIGHT].b_F[ZZ] > touch_down_fz_thres)
      {
        _doToeOff = false;
        printf("right fz %g\n", rs.feet[RIGHT].b_F[ZZ]);
        
        _lastFeet[RIGHT] = Eigen::Map <const Eigen::Vector3d> (rs.feet[RIGHT].w_sensor_pos); 

        _com_int_flag[LEFT] = false;
        // has new steps
        if (procNextStep(rs)) {
          if (_stanceFoot == LEFT) {
            setupComDs(rs, LEFT, LEFT); 
            gotoState(rs, ShiftComToLeft);
          }
          else if (_stanceFoot == RIGHT) {
            setupComDs(rs, LEFT, RIGHT); 
            gotoState(rs, ShiftComToRight);
          }
        }
        else {
          gotoState(rs, Idle);
          setupComDs(rs, LEFT, -1); 
        }
      }
      break;
  }

  if (!idCon->ID(rs, idcmd, cmd))
    return -1;
  
  if (!ikCon->IK(ikcmd, cmd))
    return -1;

  // integrate com offset towards cop mid
  for (int side = 0; side < LR; side++) {
    if (_com_int_flag[side]) {
      Eigen::Vector2d body_frame = w_cop2b_cop(rs, side);

      _com_d_i[side][XX] += com_int_i[XX]*(_com_d_over_foot[XX] - body_frame[XX]);
      _com_d_i[side][YY] += com_int_i[YY]*(_com_d_over_foot[YY] - body_frame[YY]);

      clamp(_com_d_i[side][XX], -0.1, 0.1);
      clamp(_com_d_i[side][YY], -0.1, 0.1);
    }
  }

  return 0;
}

Eigen::Vector2d AtlasStaticWalkingCon::w_cop2b_cop(const RobotState &rs, int side)
{
  Eigen::Vector2d w_cop(rs.cop[0]-rs.feet[side].w_sensor_pos[0], rs.cop[1]-rs.feet[side].w_sensor_pos[1]);
  return rotate2d(w_cop, -getYaw(rs.feet[side].w_q)); 
}

bool AtlasStaticWalkingCon::procNextStep(const RobotState &rs)
{
  if (_footSteps.empty()) 
    return false;

  _nxtFootStep = _footSteps.front();
  if (_nxtFootStep.type == SSL) {
    _stanceFoot = LEFT;
    _swingFoot = RIGHT;
  }
  else if (_nxtFootStep.type == SSR) {
    _stanceFoot = RIGHT;
    _swingFoot = LEFT;
  }
  else {
    _footSteps.pop_front(); 
    return false;
  }
  
  // remove head 
  _footSteps.pop_front(); 
  _stepCount++;
  
  if (_nxtFootStep.type == SSL)
    _nxtFootStep.pose.pos[2] += _lastFeet[LEFT][ZZ];
  else
    _nxtFootStep.pose.pos[2] += _lastFeet[RIGHT][ZZ]; 

  printf("side %d, lz %g, %g %g %g\n", _swingFoot, _lastFeet[_swingFoot][ZZ], _nxtFootStep.pose.pos[0], _nxtFootStep.pose.pos[1], _nxtFootStep.pose.pos[2]);

  return true;
}

void AtlasStaticWalkingCon::interp(const RobotState &rs, TrajPoint <12,0> &com_d)
{
  double t0 = _com0[0].time;
  double t1 = _com1[0].time;
  double slerpt = (rs.time-t0) / (t1-t0);
  slerpt = std::min(slerpt, 1.);
  slerpt = std::max(slerpt, 0.);

  Eigen::Map <Eigen::Quaterniond> tmpQ_p(com_d.pos+3);
  Eigen::Map <Eigen::Quaterniond> tmpQ_t(com_d.pos+8);
  Eigen::Map <Eigen::Vector3d>    tmpW_p(com_d.vel+3);
  Eigen::Map <Eigen::Vector3d>    tmpW_t(com_d.vel+8);

  tmpQ_p = _pelvQ0.slerp(slerpt, _pelvQ1).normalized();
  Eigen::AngleAxisd root_wQ = Eigen::AngleAxisd(_pelvQ1*(_pelvQ0.inverse()));
  if (slerpt < 1)
    tmpW_p = root_wQ.axis()*(root_wQ.angle()/(t1-t0)); 
  else
    tmpW_p = Eigen::Vector3d::Zero();

  tmpQ_t = _torsoQ0.slerp(slerpt, _torsoQ1).normalized();
  root_wQ = Eigen::AngleAxisd(_torsoQ1*(_torsoQ0.inverse()));
  if (slerpt < 1)
    tmpW_t = root_wQ.axis()*(root_wQ.angle()/(t1-t0)); 
  else
    tmpW_t = Eigen::Vector3d::Zero();

  lookup_pos_only(rs.time, _com0, _com1, com_d.pos, com_d.vel, com_d.acc);

  quintic_spline(rs.time, &_wl0, &_wl1, com_d.pos+7, NULL, NULL);
}

void AtlasStaticWalkingCon::interpFoot(const RobotState &rs, TrajPoint <7,0> &foot_d)
{
  double t0 = _swgFoot0[0].time;
  double t1 = _swgFoot1[0].time;
  double slerpt = (rs.time-t0) / (t1-t0);
  slerpt = std::min(slerpt, 1.);
  slerpt = std::max(slerpt, 0.);
  
  Eigen::Map <Eigen::Quaterniond> tmpQ(foot_d.pos+3);
  Eigen::Map <Eigen::Vector3d> tmpW(foot_d.vel+3);
  
  tmpQ = _swgQ0.slerp(slerpt, _swgQ1).normalized();
  Eigen::AngleAxisd wQ = Eigen::AngleAxisd(_swgQ1*(_swgQ0.inverse()));
  if (slerpt < 1)
    tmpW = wQ.axis()*(wQ.angle()/(t1-t0));
  else 
    tmpW = Eigen::Vector3d::Zero();

  //printf("0 (%g %g %g), 1 (%g %g %g)\n", _swgFoot0[0].pos, _swgFoot0[1].pos, _swgFoot0[2].pos, _swgFoot1[0].pos, _swgFoot1[1].pos, _swgFoot1[2].pos);

  lookup_pos_only(rs.time, _swgFoot0, _swgFoot1, foot_d.pos, foot_d.vel, foot_d.acc);
}

void AtlasStaticWalkingCon::setupSwingLiftoff(const RobotState &rs)
{
  double pos0[9] = {0}, pos1[9] = {0};
  double t0 = rs.time;
  double t1;
  
  double z_d = liftoff_height;
  double dz = _nxtFootStep.pose.pos[2] - _lastFeet[_swingFoot][ZZ];

  printf("side %d, lz %g, %g %g %g\n", _swingFoot, _lastFeet[_swingFoot][ZZ], _nxtFootStep.pose.pos[0], _nxtFootStep.pose.pos[1], _nxtFootStep.pose.pos[2]);
  
  // pull it out from IK
  if (_doToeOff) {
    dvec_copy(pos0, ikCon->ikrs->feet[_swingFoot].w_toe_pos, 3);
    dvec_copy(pos1, ikCon->ikrs->feet[_swingFoot].w_toe_pos, 3);
  }
  else {
    dvec_copy(pos0, ikCon->ikrs->feet[_swingFoot].w_sensor_pos, 3);
    dvec_copy(pos1, ikCon->ikrs->feet[_swingFoot].w_sensor_pos, 3);
  }
  
  _liftoffDuration = std::max(1.5, fabs(z_d) / liftoff_rate);
  t1 = t0 + _liftoffDuration; 

  // joint control for this phase
  for (int i = 0; i < 6; i++) {
    double j[2]; 
    int idx = A_L_LEG_UHZ+i+6*_swingFoot;
    j[0] = j[1] = ikCon->ikrs->joints[idx];
    if (i == 2)
      j[1] -= 0.2;
    if (i == 3)
      j[1] += 0.5;
    if (i == 4)
      j[1] = -0.8;

    set_single_knot(t0, t1, j[0], j[1], _joints0_d+idx, _joints1_d+idx);
  }

  _swingLegJointMode = true;

  /*
  _swgQ0 = ikCon->ikrs->feet[_swingFoot].w_q;
  _swgQ1 = _swgQ0; //rs.feet[_swingFoot].w_q;
  
  pos1[2] += z_d;
  _liftoffDuration = std::max(1., fabs(z_d) / liftoff_rate);
  t1 = t0 + _liftoffDuration; 
  set_knots_pos_only(t0, t1, pos0, pos1, pos0+3, pos1+3, pos0+6, pos1+6, _swgFoot0, _swgFoot1);
  */
}

void AtlasStaticWalkingCon::setupSwingUp(const RobotState &rs)
{
  double pos0[9] = {0}, pos1[9] = {0};
  double t0 = rs.time;
  double t1 = t0;
  double slerpt; 

  _swingLegJointMode = false;

  for (int i = 0; i < 3; i++)
    pos0[i] = ikCon->ikrs->feet[_swingFoot].w_sensor_pos[i];
  dvec_copy(pos1, _nxtFootStep.pose.pos.data(), 3);
  pos1[2] += ss_height;
  pos1[2] = std::max(pos0[2], pos1[2]);
     
  Eigen::Vector3d diff = Eigen::Map<Eigen::Vector3d>(pos0) - Eigen::Map<Eigen::Vector3d>(pos1);
  double dt = diff.norm() / swing_rate;
  dt = std::max(1., dt);
  t1 += dt;

  _swgQ0 = ikCon->ikrs->feet[_swingFoot].w_q;
  _swgQ1 = zyx2quat(Eigen::Vector3d(0, -0.1, getYaw(_nxtFootStep.pose.rot)));

  printf("swg up, pos0 %g %g %g, pos1 %g %g %g\n", pos0[0], pos0[1], pos0[2], pos1[0], pos1[1], pos1[2]);

  set_knots_pos_only(t0, t1, pos0, pos1, pos0+3, pos1+3, pos0+6, pos1+6, _swgFoot0, _swgFoot1);
}
 
void AtlasStaticWalkingCon::setupSwingDown(const RobotState &rs)
{
  for (int i = 0; i < 3; i++) {
    _swgFoot0[i].pos = _swgFoot1[i].pos;
    _swgFoot0[i].time = rs.time;
    _swgFoot1[i].pos = _nxtFootStep.pose.pos[i];
    _swgFoot1[i].time = rs.time + 0.5*ss_duration;

    _swgFoot0[i].acc = 0;
    _swgFoot0[i].vel = 0;
    _swgFoot1[i].acc = 0;
    _swgFoot1[i].vel = 0;
  }
  _swgFoot1[2].pos += 0.02; 
  _swgQ0 = ikCon->ikrs->feet[_swingFoot].w_q; //rs.feet[RIGHT].w_q;
  _swgQ1 = _nxtFootStep.pose.rot;
}

Eigen::Vector3d AtlasStaticWalkingCon::computeCom_d(const RobotState &rs, int side, double dz)
{
  Eigen::Vector3d ret;

  if (side == -1) {
    for (int i = 0; i < 3; i++)
      ret[i] = (rs.feet[LEFT].w_mid_pos[i] + rs.feet[RIGHT].w_mid_pos[i]) / 2.;
  }
  else {
    double dx[2] = {_com_d_i[side][XX], _com_d_i[side][YY]};
    _com_d_over_foot[XX] = Foot::ankle_to_center_offset[XX];
    _com_d_over_foot[YY] = Foot::ankle_to_center_offset[YY];

    double yaw = getYaw(rs.feet[side].w_q);
    double offset[3] = {cos(yaw)*dx[0] - sin(yaw)*dx[1], sin(yaw)*dx[0] + cos(yaw)*dx[1], 0};

    for (int i = 0; i < 3; i++)
      ret[i] = rs.feet[side].w_mid_pos[i] + offset[i];
  }
  
  // fix z
  ret[ZZ] += com_z;

  return ret;
}

void AtlasStaticWalkingCon::updateKnot(const RobotState &rs)
{
  //////////////////////////////////////////////
  // com
  Eigen::Vector3d com_d = computeCom_d(rs, _shiftCOMend, 0);
  for (int i = 0; i < 3; i++) {
    _com1[i].pos = com_d[i];
    _com1[i].vel = 0;
    _com1[i].acc = 0;
    _com0[i].vel = 0;
    _com0[i].acc = 0;
  }


  //////////////////////////////////////////////
  // foot
  static double dz_hack = 0;
  if (_state == SwingDownLeft) {
    if (rs.time > _swgFoot1[2].time) {
      dz_hack += ss_lower_z_vel * rs.timeStep;
      _swgFoot1[2].vel = ss_lower_z_vel;
    }
    else {
      dz_hack = 0;
      _swgFoot1[2].vel = 0;
    }
    _swgFoot1[2].pos = _nxtFootStep.pose.pos[2] + 0.02 + dz_hack;
  }
  else if (_state == SwingDownRight) {
    if (rs.time > _swgFoot1[2].time) {
      dz_hack += ss_lower_z_vel * rs.timeStep;
      _swgFoot1[2].vel = ss_lower_z_vel;
    }
    else {
      dz_hack = 0;
      _swgFoot1[2].vel = 0;
    }
    _swgFoot1[2].pos = _nxtFootStep.pose.pos[2] + 0.02 + dz_hack; 
  }
}

bool AtlasStaticWalkingCon::setupComSs(const RobotState &rs)
{
  double pos0[9] = {0}, pos1[9] = {0};
  double t0 = rs.time;
  double t1 = t0 + ss_duration + _liftoffDuration;
  double slerpt;
  
  // setup orientation knots
  _pelvQ0 = _pelvQ1;
  _torsoQ0 = _torsoQ1;

  _torsoQ1 = zyx2quat(Eigen::Vector3d(0, 0., getYaw(rs.feet[_stanceFoot].w_q)));
  _pelvQ1 = zyx2quat(Eigen::Vector3d(0, 0., getYaw(rs.feet[_stanceFoot].w_q)));

  if (_pelvQ1.dot(_pelvQ0) < 0)
    neg_quat(_pelvQ1);
  if (_torsoQ1.dot(_torsoQ0) < 0)
    neg_quat(_torsoQ1);

  // use the last planned
  for (int i = 0; i < 3; i++) {
    pos0[i] = _com1[i].pos;
  }

  dvec_copy(pos1, rs.feet[_stanceFoot].w_mid_pos, 3);
  pos1[ZZ] += com_z;
  //pos1[ZZ] += com_z - 0.03;

  set_knots_pos_only(t0, t1, pos0, pos1, pos0+3, pos1+3, pos0+6, pos1+6, _com0, _com1);
 
  return true; 
}

bool AtlasStaticWalkingCon::setupComDs(const RobotState &rs, int start, int end)
{
  double pos0[9] = {0}, pos1[9] = {0};
  double t0 = rs.time;
  double t1 = t0 + ds_duration;
  double slerpt;

  double wl0, wl1;

  _shiftCOMstart = start;
  _shiftCOMend = end;
  
  // setup orientation knots
  _pelvQ0 = _pelvQ1;
  _torsoQ0 = _torsoQ1;

  double foot_yaw[2] = {getYaw(rs.feet[LEFT].w_q), getYaw(rs.feet[RIGHT].w_q)};
  double yaw_avg = avgAngle(foot_yaw[0], foot_yaw[1]);
  //double yaw_avg = (foot_yaw[0] + foot_yaw[1]) / 2.;

  _torsoQ1 = zyx2quat(Eigen::Vector3d(0, 0., yaw_avg));
  _pelvQ1 = zyx2quat(Eigen::Vector3d(0, 0., yaw_avg));

  if (_pelvQ1.dot(_pelvQ0) < 0)
    neg_quat(_pelvQ1);
  if (_torsoQ1.dot(_torsoQ0) < 0)
    neg_quat(_torsoQ1);

  // setup pos knots
  for (int i = 0; i < 3; i++) {
    pos0[i] = _com1[i].pos;
  }

  // set end position, not exactly correct, but will get overriden
  // in updateKnots anyway
  if (end == LEFT) {
    dvec_copy(pos1, rs.feet[LEFT].w_mid_pos, 3);
    _com_d_i[LEFT][YY] = std::min(_com_d_i_ini[LEFT][YY], _com_d_i[LEFT][YY]);
  }
  else if (end == RIGHT) {
    dvec_copy(pos1, rs.feet[RIGHT].w_mid_pos, 3);
    _com_d_i[RIGHT][YY] = std::max(_com_d_i_ini[RIGHT][YY], _com_d_i[RIGHT][YY]);
  }
  else {
    for (int i = 0; i < 3; i++) 
      pos1[i] = (rs.feet[LEFT].w_mid_pos[i]+rs.feet[RIGHT].w_mid_pos[i]) / 2.;
  }
  pos1[ZZ] += com_z + 0.05;

  set_knots_pos_only(t0, t1, pos0, pos1, pos0+3, pos1+3, pos0+6, pos1+6, _com0, _com1);

  for (int i = 0; i < 3; i++) {
    printf("pos 0 %g, 1 %g\n", _com0[i].pos, _com1[i].pos); 
  }

  // set wl
  wl0 = _wl1.pos;
  
  if (end == LEFT)
    wl1 = 1;
  else if (end == RIGHT)
    wl1 = 0;
  else 
    wl1 = 0.5;

  set_single_knot(t0, t1, wl0, wl1, &_wl0, &_wl1);

  return true;
}

void AtlasStaticWalkingCon::fillDesireds(const RobotState &rs, const TrajPoint<12,0> &root_d, const TrajPoint<7,0> foot_d[2])
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
      quintic_spline(rs.time, _joints0_d+i, _joints1_d+i, ikcmd.joints+i, ikcmd.jointsd+i, idcmd.jointsdd+i);
      idcmd.jointsdd[i] += fd_joint_p*(ikcmd.joints[i]-rs.joints[i]) 
        + fd_joint_d*(ikcmd.jointsd[i]-rs.jointsd[i]); 
      
      if (i == A_NECK_AY) {
        ikcmd.joints[i] = _neck_ay_d;
        ikcmd.jointsd[i] = 0;
        idcmd.jointsdd[i] = 0;
      }
    }
  }

  if (_swingLegJointMode) {
    ikcmd.skipFoot[_swingFoot] = true;
    idcmd.skipFoot[_swingFoot] = true;
    for (int i = 0; i < 6; i++) {
      int j = A_L_LEG_UHZ+6*_swingFoot+i;
      quintic_spline(rs.time, _joints0_d+j, _joints1_d+j, ikcmd.joints+j, ikcmd.jointsd+j, idcmd.jointsdd+j);
      idcmd.jointsdd[j] += fd_joint_p*(ikcmd.joints[j]-rs.joints[j]) + fd_joint_d*(ikcmd.jointsd[j]-rs.jointsd[j]);
    }
  }
  else {
    ikcmd.skipFoot[LEFT] = false;
    idcmd.skipFoot[LEFT] = false;
    ikcmd.skipFoot[RIGHT] = false;
    idcmd.skipFoot[RIGHT] = false;
  }
  
  //////////////////////////////////////////////////////////////////////////////////////////
  // root 
  for (int i = 0; i < 3; i++) {
    ikcmd.com[i] = root_d.pos[i];
    ikcmd.comd[i] = root_d.vel[i];
    idcmd.comdd[i] = root_d.acc[i] 
      + fd_com_p[i]*(ikcmd.com[i]-rs.com[i]) 
      + 2*fd_com_d[i]*sqrt(fd_com_p[i])*(ikcmd.comd[i]-rs.comd[i]);

    ikcmd.root[i] = INFINITY; 
    ikcmd.rootd[i] = INFINITY;
    idcmd.rootdd[i] = INFINITY;
  }

  // hack pelvis z height
  ikcmd.rootMode[ZZ] = idcmd.rootMode[ZZ] = true;
  idcmd.comdd[ZZ] = root_d.acc[ZZ] 
    + fd_com_p[ZZ]*(ikcmd.com[ZZ]-rs.root[ZZ]) 
    + 2*fd_com_d[ZZ]*sqrt(fd_com_p[ZZ])*(ikcmd.comd[ZZ]-rs.rootd[ZZ]);
  for (int i = 0; i < 3; i++)
    clamp(idcmd.comdd[i], -0.5, 0.5);

  // orientation tracking
  ikcmd.torsoQ = Eigen::Map<const Eigen::Quaterniond> (root_d.pos+8);
  if (ikcmd.torsoQ.dot(rs.utorsoq) < 0)
    neg_quat(ikcmd.torsoQ);
  
  Eigen::Vector3d Rotvec = quatMinus(rs.utorsoq, ikcmd.torsoQ);

  for (int i = 0; i < 3; i++) {
    ikcmd.torsoW[i] = root_d.vel[i+8];
    idcmd.torsoWd[i] = 0 
      - fd_com_p[i+3]*(Rotvec[i]) 
      + 2*fd_com_d[i+3]*sqrt(fd_com_p[i+3])*(ikcmd.torsoW[i]-rs.utorsow[i]);
  }  
  for (int i = 0; i < 3; i++)
    clamp(idcmd.torsoWd[i], -0.5, 0.5);

  // root orientation tracking 
  ikcmd.pelvQ = Eigen::Map<const Eigen::Quaterniond> (root_d.pos+3);
  if (ikcmd.pelvQ.dot(rs.rootq) < 0)
    neg_quat(ikcmd.pelvQ);
  
  Rotvec = quatMinus(rs.rootq, ikcmd.pelvQ);

  for (int i = 0; i < 3; i++) {
    ikcmd.pelvW[i] = root_d.vel[i+3];
    idcmd.pelvWd[i] = 0 
      - fd_com_p[i+3]*(Rotvec[i]) 
      + 2*fd_com_d[i+3]*sqrt(fd_com_p[i+3])*(ikcmd.pelvW[i]-rs.rootd[i+3]);
  }
  for (int i = 0; i < 3; i++)
    clamp(idcmd.pelvWd[i], -0.5, 0.5);

  /////////////////////////////////////////////////////////////
  // reset "history"
  // this is for the integrate towards real foot stuff in stance
  static Foot::FootRefType lastRef[LR] = {(Foot::FootRefType)-1, (Foot::FootRefType)-1};
  for (int side = 0; side < LR; side++) {
    if (lastRef[side] != ikcmd.footRef[side]) {
      lastRef[side] = ikcmd.footRef[side];
      ikcmd.footQ[side] = rs.feet[side].w_q;
      switch (ikcmd.footRef[side]) {
        case Foot::RefCenter:
          dvec_copy(ikcmd.foot[side], rs.feet[side].w_mid_pos, 3);
          break;
        case Foot::RefToe:
          dvec_copy(ikcmd.foot[side], rs.feet[side].w_toe_pos, 3);
          break;
        case Foot::RefHeel:
          dvec_copy(ikcmd.foot[side], rs.feet[side].w_heel_pos, 3);
          break;
        case Foot::RefSensor:
          dvec_copy(ikcmd.foot[side], rs.feet[side].w_sensor_pos, 3);
          break;
      }
    }
  }
  
  // switch which point for feedback based on ref
  const double *foot_pos[LR] = {NULL, NULL};
  const double *foot_vel[LR] = {NULL, NULL};

  for (int side = 0; side < LR; side++) {
    switch (ikcmd.footRef[side]) {
      case Foot::RefCenter:
        foot_pos[side] = rs.feet[side].w_mid_pos;
        foot_vel[side] = rs.feet[side].w_mid_vel;
        break;
      case Foot::RefToe:
        foot_pos[side] = rs.feet[side].w_toe_pos;
        foot_vel[side] = rs.feet[side].w_toe_vel;
        break;
      case Foot::RefHeel:
        foot_pos[side] = rs.feet[side].w_heel_pos;
        foot_vel[side] = rs.feet[side].w_heel_vel;
        break;
      case Foot::RefSensor:
        foot_pos[side] = rs.feet[side].w_sensor_pos;
        foot_vel[side] = rs.feet[side].w_sensor_vel;
        break;
    }
  }

  for (int side = 0; side < LR; side++) 
  {
    // swing
    if (isSwing(rs.contactState, side)) 
    {
      if (!_swingLegJointMode) {
        dvec_copy(ikcmd.foot[side], foot_d[side].pos, 3);
        dvec_copy(_realFoot_d[side], foot_d[side].pos, 3);
        _foot_d_i[side][XX] += foot_int_i[side]*(foot_d[side].pos[XX] - rs.feet[side].w_sensor_pos[XX]);
        _foot_d_i[side][YY] += foot_int_i[side]*(foot_d[side].pos[YY] - rs.feet[side].w_sensor_pos[YY]);

        clamp(_foot_d_i[side][XX], -0.1, 0.1);
        clamp(_foot_d_i[side][YY], -0.1, 0.1);

        ikcmd.foot[side][XX] += _foot_d_i[side][XX];
        ikcmd.foot[side][YY] += _foot_d_i[side][YY];

        // add an offset because there is tracking error for pelvis
        //for (int i = 0; i < 3; i++) 
        //  ikcmd.foot[side][i] += ikCon->ikrs->root[i] - rs.root[i];

        ikcmd.footQ[side] = Eigen::Map<const Eigen::Quaterniond> (foot_d[side].pos+3);

        for (int j = 0; j < 3; j++) {
          ikcmd.footd[side][j] = foot_d[side].vel[j];
          idcmd.footdd[side][j] = foot_d[side].acc[j]
            + fd_foot_p[j]*(ikcmd.foot[side][j]-foot_pos[side][j])
            + 2*fd_foot_d[j]*sqrt(fd_foot_p[j])*(ikcmd.footd[side][j]-foot_vel[side][j]);
        }
        if (ikcmd.footQ[side].dot(rs.feet[side].w_q) < 0) 
          neg_quat(ikcmd.footQ[side]);

        Rotvec = quatMinus(rs.feet[side].w_q, ikcmd.footQ[side]);

        for (int i = 3; i < 6; i++) {
          ikcmd.footd[side][i] = foot_d[side].vel[i];
          idcmd.footdd[side][i] = 0 
            - fd_foot_p[i]*Rotvec[i-3]
            + 2*fd_foot_d[i]*sqrt(fd_foot_p[i])*(ikcmd.footd[side][i]-foot_vel[side][i]);
        }
      }
    }
    else {
      // stance
      // gradually move the foot only when here is big force on it
      if (rs.feet[side].w_F[ZZ] > 200) {
        for(int i = 0; i < 3; i++) {
          ikcmd.foot[side][i] = 0.03*foot_d[side].pos[i] + 0.97*ikcmd.foot[side][i];
        }
        ikcmd.footQ[side] = ikcmd.footQ[side].slerp(0.03, Eigen::Map<const Eigen::Quaterniond> (foot_d[side].pos+3));
      }
      dvec_copy(_realFoot_d[side], foot_d[side].pos, 3);
      
      for (int i = 0; i < 6; i++) {
        ikcmd.footd[side][i] = 0;
        idcmd.footdd[side][i] = 0;
      }
      /*
      for (int i = 0; i < 3; i++) {
        ikcmd.footd[side][i] = foot_d[side].vel[i];
        idcmd.footdd[side][i] = foot_d[side].acc[i]
          + fd_foot_p[i]*(ikcmd.foot[side][i]-foot_pos[side][i])
          + fd_foot_d[i]*(ikcmd.footd[side][i]-foot_vel[side][i]);
        //fprintf(stderr, "%d %g %g %g\n", i, ikcmd.footd[side][i], idcmd.footdd[side][i]);
      }
      if (ikcmd.footQ[side].dot(rs.feet[side].w_q) < 0) 
        neg_quat(ikcmd.footQ[side]);

      Rotvec = quatMinus(rs.feet[side].w_q, ikcmd.footQ[side]);

      for (int i = 3; i < 6; i++) {
        ikcmd.footd[side][i] = foot_d[side].vel[i];
        idcmd.footdd[side][i] = 0 
          - fd_foot_p[i]*Rotvec[i-3]
          + fd_foot_d[i]*(ikcmd.footd[side][i]-foot_vel[side][i]);
        //fprintf(stderr, "%d %g %g %g\n", i, ikcmd.footd[side][i], idcmd.footdd[side][i]);
      }
      */
    }
    for (int i = 0; i < 6; i++)
      clamp(idcmd.footdd[side][i], -0.5, 0.5);

    /*
    // tell IK it can't use the ankle joints
    if (_footStage[side] == WalkingCon::Planted) {
      ikcmd.stanceFoot[side] = true;
      ikcmd.joints[A_L_LEG_UAY+6*side] = rs.joints[A_L_LEG_UAY+6*side];
      ikcmd.joints[A_L_LEG_LAX+6*side] = rs.joints[A_L_LEG_LAX+6*side];
      ikcmd.jointsd[A_L_LEG_UAY+6*side] = rs.jointsd[A_L_LEG_UAY+6*side];
      ikcmd.jointsd[A_L_LEG_LAX+6*side] = rs.jointsd[A_L_LEG_LAX+6*side];
    }
    else 
      ikcmd.stanceFoot[side] = false;
    */
  }

  switch (rs.contactState) {
    case DSl:
    case DSr:
    case DSc:
      //yaw_avg = (getYaw(rs.feet[LEFT].w_q) + getYaw(rs.feet[RIGHT].w_q)) / 2.; 
      idcmd.wl = root_d.pos[7];
      _cop_comp_z = (rs.feet[LEFT].w_sensor_pos[ZZ]*rs.feet[LEFT].w_F[ZZ] + rs.feet[RIGHT].w_sensor_pos[ZZ]*rs.feet[RIGHT].w_F[ZZ]) / (rs.feet[LEFT].w_F[ZZ]+rs.feet[RIGHT].w_F[ZZ]);
      // z = (rs.feet[LEFT].w_sensor_pos[ZZ] + rs.feet[RIGHT].w_sensor_pos[ZZ]) / 2.;
      break;
    case SSL:
      //yaw_avg = getYaw(rs.feet[LEFT].w_q);
      idcmd.wl = 1;
      _cop_comp_z = rs.feet[LEFT].w_sensor_pos[ZZ];
      break;
    case SSR:
      //yaw_avg = getYaw(rs.feet[RIGHT].w_q);
      idcmd.wl = 0;
      _cop_comp_z = rs.feet[RIGHT].w_sensor_pos[ZZ];
      break;
    default:
      assert(0);
  }
   
  //Eigen::Matrix2d rot;
  //rot << cos(yaw_avg), -sin(yaw_avg), sin(yaw_avg), cos(yaw_avg); 
  
  // fix cop stuff
  for (int i = 0; i < 2; i++)  
    idcmd.cop[i] = rs.com[i] - idcmd.comdd[i] * (rs.com[ZZ] - _cop_comp_z) / GRAVITY;

  // don't split cop anymore, just regularize them to mid
  for (int side = 0; side < LR; side++) {
    if (idcmd.footRef[side] == Foot::RefToe)
      idcmd.b_cop[side][XX] = 0.0;
    else {
      idcmd.b_cop[side][XX] = 0.045; //hack_cop(rs.joints[A_L_LEG_KNY + 6*side]);
    }
    idcmd.b_cop[side][YY] = 0.;
  }

  if (_state == ShiftComToRight)
    idcmd.b_cop[LEFT][XX] = hack_cop(rs.joints[A_L_LEG_KNY]);
  if (_state == ShiftComToLeft)
    idcmd.b_cop[RIGHT][XX] = hack_cop(rs.joints[A_R_LEG_KNY]);

  /*
  splitCoP(idcmd.cop, idcmd.wl, rs.feet[LEFT].w_sensor_pos, rs.feet[RIGHT].w_sensor_pos, idcmd.w_cop[LEFT], idcmd.w_cop[RIGHT]);
  
  for (int side = 0; side < LR; side++) {
    Eigen::Vector2d tmp(idcmd.w_cop[side]);
    yaw_avg = getYaw(rs.feet[side].w_q);
    Eigen::Matrix2d world_to_foot;
    world_to_foot(0,0) = cos(-yaw_avg);
    world_to_foot(0,1) = -sin(-yaw_avg);
    world_to_foot(1,0) = sin(-yaw_avg);
    world_to_foot(1,1) = cos(-yaw_avg);
    Eigen::Map <Eigen::Vector2d> b_cop(idcmd.b_cop[side]);
    b_cop = world_to_foot * tmp;

    if (idcmd.footRef[side] == Foot::RefToe)
      b_cop[XX] = 0;
    
    // add back the pos from foot, for book keeping, not in ID
    idcmd.w_cop[side][XX] += rs.feet[side].w_sensor_pos[XX];
    idcmd.w_cop[side][YY] += rs.feet[side].w_sensor_pos[YY];
  }
  */
}

static double hack_cop(double kny) 
{
  double s = (kny - 1.13) / (0.8-1.13);
  double cop = 0.045 + s * (0.175-0.045);
  clamp(cop, -0.085, 0.175);

  return cop;
}

void AtlasStaticWalkingCon::buildLookup()
{
  std::string name;

  _conName = "ASW";

  _lookup["fd_joint_p"] = &fd_joint_p;
  _lookup["fd_joint_d"] = &fd_joint_d;

  _lookup["com_z"] = &com_z;
  _lookup["ds_duration"] = &ds_duration;
  _lookup["ss_duration"] = &ss_duration;
  _lookup["swing_rate"] = &swing_rate;
  _lookup["liftoff_rate"] = &liftoff_rate;
  _lookup["liftoff_height"] = &liftoff_height;
  _lookup["ss_height"] = &ss_height;
  _lookup["ss_lower_z_vel"] = &ss_lower_z_vel;
  _lookup["com_int_i_x"] = &(com_int_i[XX]);
  _lookup["com_int_i_y"] = &(com_int_i[YY]);
  _lookup["foot_int_i_x"] = &(foot_int_i[XX]);
  _lookup["foot_int_i_y"] = &(foot_int_i[YY]);
  
  _lookup["touch_down_fz_thres"] = &touch_down_fz_thres;

  _lookup["com_on_foot_offset_l_x"] = &(_com_d_i_ini[LEFT][XX]);
  _lookup["com_on_foot_offset_l_y"] = &(_com_d_i_ini[LEFT][YY]);
  _lookup["com_on_foot_offset_r_x"] = &(_com_d_i_ini[RIGHT][XX]);
  _lookup["com_on_foot_offset_r_y"] = &(_com_d_i_ini[RIGHT][YY]);

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
}

const std::string AtlasStaticWalkingCon::conModeNames[NUM_CON_STATE] = 
{
  "Com to Left",
  "Com to Right",
  "Left Fix Cop",
  "Right Fix Cop",
  "Left Lift off",
  "Left swing up",
  "Left swing down",
  "Right Lift off",
  "Right swing up",
  "Right swing down",
  "Idle"
};
   
void AtlasStaticWalkingCon::deleteFootStep(const SFootStep &fs, int i)
{
  std::list<SFootStep>::iterator it;
  int ctr = 0;
  for (it = _footSteps.begin(); it != _footSteps.end(); it++) {
    if (ctr == i)
      break;
    ctr++;
  }
  if (ctr == i && it != _footSteps.end()) 
    _footSteps.erase(it);
}

void AtlasStaticWalkingCon::addToLog(BatchLogger &logger)
{
  logger.add_datapoint("ASW.state", "-", &(_state));
  logger.add_datapoint("ASW.footStage[L]", "-", (const int *)&(_footStage[LEFT]));
  logger.add_datapoint("ASW.footStage[R]", "-", (const int *)&(_footStage[RIGHT]));
  logger.add_datapoint("ASW.cop_comp_z", "-", &_cop_comp_z);
  logger.add_datapoint("ASW.com_d_i[L][X]", "m", &(_com_d_i[LEFT][XX]));
  logger.add_datapoint("ASW.com_d_i[L][Y]", "m", &(_com_d_i[LEFT][YY]));
  logger.add_datapoint("ASW.com_d_i[R][X]", "m", &(_com_d_i[RIGHT][XX]));
  logger.add_datapoint("ASW.com_d_i[R][Y]", "m", &(_com_d_i[RIGHT][YY]));
  logger.add_datapoint("ASW.foot_d_i[L][X]", "m", &(_foot_d_i[LEFT][XX]));
  logger.add_datapoint("ASW.foot_d_i[L][Y]", "m", &(_foot_d_i[LEFT][YY]));
  logger.add_datapoint("ASW.foot_d_i[R][X]", "m", &(_foot_d_i[RIGHT][XX]));
  logger.add_datapoint("ASW.foot_d_i[R][Y]", "m", &(_foot_d_i[RIGHT][YY]));
  
  _nxtFootStep.addToLog(logger);

  logger.add_datapoint("ASW.com0[X]", "m", &(_com0[XX].pos));
  logger.add_datapoint("ASW.com0[Y]", "m", &(_com0[YY].pos));
  logger.add_datapoint("ASW.com0[Z]", "m", &(_com0[ZZ].pos));
  logger.add_datapoint("ASW.com1[X]", "m", &(_com1[XX].pos));
  logger.add_datapoint("ASW.com1[Y]", "m", &(_com1[YY].pos));
  logger.add_datapoint("ASW.com1[Z]", "m", &(_com1[ZZ].pos));
  
  logger.add_datapoint("ASW.swgFoot0[X]", "m", &(_swgFoot0[XX].pos));
  logger.add_datapoint("ASW.swgFoot0[Y]", "m", &(_swgFoot0[YY].pos));
  logger.add_datapoint("ASW.swgFoot0[Z]", "m", &(_swgFoot0[ZZ].pos));
  logger.add_datapoint("ASW.swgFoot1[X]", "m", &(_swgFoot1[XX].pos));
  logger.add_datapoint("ASW.swgFoot1[Y]", "m", &(_swgFoot1[YY].pos));
  logger.add_datapoint("ASW.swgFoot1[Z]", "m", &(_swgFoot1[ZZ].pos));
  
  logger.add_datapoint("ASW.rf[L][X]", "m", &(_realFoot_d[LEFT][XX]));
  logger.add_datapoint("ASW.rf[L][Y]", "m", &(_realFoot_d[LEFT][YY]));
  logger.add_datapoint("ASW.rf[L][Z]", "m", &(_realFoot_d[LEFT][ZZ]));
  logger.add_datapoint("ASW.rf[R][X]", "m", &(_realFoot_d[RIGHT][XX]));
  logger.add_datapoint("ASW.rf[R][Y]", "m", &(_realFoot_d[RIGHT][YY]));
  logger.add_datapoint("ASW.rf[R][Z]", "m", &(_realFoot_d[RIGHT][ZZ]));
}
   

