/*
 * =====================================================================================
 *
 *       Filename:  Task2WalkingCon.cpp
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

#include "Task2WalkingCon.h"
#include "Eigen_utils.hpp"
#include <boost/thread.hpp>
#include <algorithm>

#define STOP        -10
#define START_LEFT  1
#define START_RIGHT 2

static double hack_cop(double kny);

void Task2WalkingCon::setUpperbodyMode(HatRobotState::UpperBodyMode m)
{
  if (ikCon->ikrs->getType() != RobotState::TYPE_HAT)
    return;

  _ubodyMode = m;
  ((HatRobotState *)ikCon->ikrs)->setUpperBodyMode(m);
}

Task2WalkingCon::Task2WalkingCon()
{
  buildLookup();

  com_z = 0.81;
  _ds_duration = 3;
  _ss_duration = 2;
  ss_height = 0.1;
  fd_joint_p = 100;
  fd_joint_d = 10;
  ss_lower_z_vel = -0.008;

  liftoff_height = 0.02;
  liftoff_rate = 0.03;
  swing_rate = 0.25;
  
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
  _ubodyMode = HatRobotState::SF;
}

Task2WalkingCon::~Task2WalkingCon()
{
}

void Task2WalkingCon::init(const RobotState &rs)
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
  _prevTerrainMode = Flat;
  _terrainMode = Flat;
  _nxtTerrainMode = Flat;
  _nxtDx = 0.1;
  _downCtr = 0;
  _downSlopeCtr = 0;
  _upCtr = 0;
  _swingLegJointMode = false;
  _com_d_over_foot[XX] = Foot::ankle_to_center_offset[XX];
  _com_d_over_foot[YY] = Foot::ankle_to_center_offset[YY];

  _ds_duration = ds_block_duration;
  _ss_duration = ss_block_duration;
  
  for (int side = 0; side < LR; side++) {
    _foot_d_i[side][XX] = 0;
    _foot_d_i[side][YY] = 0;
  }
  _com_int_flag[LEFT] = _com_int_flag[RIGHT] = false;

  for (int i = 0; i < 2; i++) {
    _rs_com0[i].pos = 0; 
    _rs_com1[i].pos = 0;
  }

  double ew_joints[N_JOINTS] = {
    0, 0, 0., 
    0, 
    0, 0, -0.23, 0.52, -0.28, 0, 
    0, 0, -0.23, 0.52, -0.28, 0., 
    0.3, -0.5, 2, 0.5, 0.2, 0, 
    0.3, 0.5, 2, -0.5, 0.2, 0 
  };

  double hm_joints[N_JOINTS] = {
    0, 0, 0., 
    0, 
    0, 0, -0.23, 0.52, -0.28, 0, 
    0, 0, -0.23, 0.52, -0.28, 0., 
    0.7, -1.3, 2, 1.8, 0.2, 0.6, 
    0.7, 1.3, 2, -1.8, 0.2, -0.6 
  };

  switch (_ubodyMode) {
    case HatRobotState::SF:
      for (int i = 0; i < N_JOINTS; i++)
        set_single_knot(rs.time, rs.time+5, rs.joints[i], Command::BDI_joints[i], _joints0_d+i, _joints1_d+i);
      break;
    case HatRobotState::EW:
      for (int i = 0; i < N_JOINTS; i++)
        set_single_knot(rs.time, rs.time+5, rs.joints[i], ew_joints[i], _joints0_d+i, _joints1_d+i);
       break;
    case HatRobotState::HM:
      for (int i = 0; i < N_JOINTS; i++)
        set_single_knot(rs.time, rs.time+5, rs.joints[i], hm_joints[i], _joints0_d+i, _joints1_d+i);
      break;      
  }

  _go = STOP;
  // script
  _scriptType = Manual;
  _script.clear();

  for (int side = 0; side < LR; side++) {
    _com_d_i[side][0] = _com_d_i_ini[side][0];
    _com_d_i[side][1] = _com_d_i_ini[side][1];
  }

  printParams();
}

void Task2WalkingCon::resetNxtOffset(int nxtSwing)
{
  _nxtOffset[0] = _nxtDx;

  if (nxtSwing == LEFT)
    _nxtOffset[1] = 0.2;
  else
    _nxtOffset[1] = -0.2;
  
  _nxtOffset[2] = 0;

  // rpy
  _nxtOffset[3] = 0;
  _nxtOffset[4] = 0;
  _nxtOffset[5] = 0; 

  if (_terrainMode == WalkingCon::UpBlock && _upCtr == 1)
    _nxtOffset[2] += 0.145;
  if (_terrainMode == WalkingCon::DownBlock && _downCtr == 1)
    _nxtOffset[2] += -0.145;
  
  if (_terrainMode == WalkingCon::UpSlope) {
    _nxtOffset[2] += 0.05;
    _nxtOffset[4] = -0.26;
  }
  if (_terrainMode == WalkingCon::DownSlope) {
    if (_downSlopeCtr == 1)
      _nxtOffset[2] += 0.08;
    _nxtOffset[4] = 0.2;
  }
}

void Task2WalkingCon::updateNxtStep()
{
  if (_stanceFoot == LEFT) {
    _swingFoot = RIGHT;
    _nxtFootStep.type = SSL;
  }
  else {
    _swingFoot = LEFT;
    _nxtFootStep.type = SSR;
  }

  Eigen::Vector2d off = rotate2d(Eigen::Vector2d(_nxtOffset[0], _nxtOffset[1]), _curHeading);

  _nxtFootStep.pose.pos = _curStanceFootPose.pos + Eigen::Vector3d(off.x(), off.y(), _nxtOffset[2]);
  _nxtFootStep.pose.rot = zyx2quat(Eigen::Vector3d(_nxtOffset[3], _nxtOffset[4], _curHeading+_nxtOffset[5])); 
}

void Task2WalkingCon::nudgeNxtStep(const double delta[6])
{
  if (_state != HoldLeftFoot && _state != HoldRightFoot && 
      _state != SwingUpLeft  && _state != SwingUpRight  &&
      _state != SwingDownLeft && _state != SwingDownRight)
    return;

  for (int i = 0; i < 6; i++) {
    if (i != 2)
      _nxtOffset[i] += delta[i];
  }

  clamp(_nxtOffset[0], -0.2, 0.4);
  if (_swingFoot == LEFT)
    clamp(_nxtOffset[1], 0.16, 0.3);
  else 
    clamp(_nxtOffset[1], -0.3, -0.16);
  
  clamp(_nxtOffset[2], -0.2, 0.2);

  clamp(_nxtOffset[3], -0.3, 0.3);
  clamp(_nxtOffset[4], -0.3, 0.3);
  if (_swingFoot == LEFT)
    clamp(_nxtOffset[5], -0.1, 0.3);
  else
    clamp(_nxtOffset[5], -0.3, 0.1);
 
  fprintf(stderr, "_nxtOffset (%g %g %g) (%g %g %g)\n", _nxtOffset[0], _nxtOffset[1], _nxtOffset[2], _nxtOffset[3], _nxtOffset[4], _nxtOffset[5]);
  printf("_nxtOffset (%g %g %g) (%g %g %g)\n", _nxtOffset[0], _nxtOffset[1], _nxtOffset[2], _nxtOffset[3], _nxtOffset[4], _nxtOffset[5]);

  updateNxtStep();
 
  //////////////////////////////////////////////
  // nudging
  if (_state == HoldLeftFoot || _state == HoldRightFoot ||
      _state == SwingUpLeft  || _state == SwingUpRight  ||
      _state == SwingDownLeft || _state == SwingDownRight) {
    for (int i = 0; i < 2; i++)
      _swgFoot1[i].pos = _nxtFootStep.pose.pos[i];
    _swgFoot1[2].pos += delta[2];

    _swgQ1 = _nxtFootStep.pose.rot;
  }
}

SFootStep Task2WalkingCon::getNextStep() const 
{ 
  return _nxtFootStep; 
}

void Task2WalkingCon::toggleWalking(int side)
{
  if (_go != STOP) {
    fprintf(stderr, "stop walking\n");  
    printf("stop walking\n");
    _go = STOP;
  }
  else if (side == LEFT) {
    fprintf(stderr, "start walking from LEFT swing\n");  
    printf("start walking from LEFT swing\n");
    _go = START_LEFT;
  }
  else if (side == RIGHT) {
    fprintf(stderr, "start walking from RIGHT swing\n");  
    printf("start walking from RIGHT swing\n");
    _go = START_RIGHT;
  }
}

void Task2WalkingCon::resetAfterStep(const RobotState &rs)
{
  if (_scriptType != Manual) {
    if (_script.empty()) {
      _nxtTerrainMode = Flat;
      _nxtDx = 0.1;
      _go = STOP;
      _scriptType = Manual;
    }
    else {
      _nxtTerrainMode = _script.front().mode;
      _nxtDx = _script.front().dx;
      _script.pop_front();
    }
  }

  _prevTerrainMode = _terrainMode;
  _terrainMode = _nxtTerrainMode;

  if (_terrainMode == WalkingCon::UpBlock) {
    _upCtr++;
    if (_upCtr > 2)
      _upCtr = 1;
  }
  else
    _upCtr = 0;

  if (_terrainMode == WalkingCon::DownBlock) {
    _downCtr++;
    if (_downCtr > 2)
      _downCtr = 1;
  }
  else
    _downCtr = 0;

  if (_terrainMode == WalkingCon::DownSlope) {
    _downSlopeCtr++;
  }
  else
    _downSlopeCtr = 0;

  _takeStep = false;
  _curHeading += _nxtOffset[5];
  std::swap(_swingFoot, _stanceFoot);
  _curStanceFootPose.pos = Eigen::Map<const Eigen::Vector3d>(rs.feet[_stanceFoot].w_sensor_pos);
  _curStanceFootPose.rot = rs.feet[_stanceFoot].w_q;
  
  resetNxtOffset(_swingFoot);
  updateNxtStep();
}

void Task2WalkingCon::setTerrainMode(WalkingConTerrainMode mode)
{
  _nxtTerrainMode = mode;
  fprintf(stderr, "nxt step mode %s\n", TerrainModeNames[_nxtTerrainMode].c_str());
  printf("nxt step mode %s\n", TerrainModeNames[_nxtTerrainMode].c_str());
}

void Task2WalkingCon::takeStep()
{
  Eigen::Vector3d zyx = quat2zyx(_nxtFootStep.pose.rot);
  fprintf(stderr, "go %g %g %g, %g %g %g\n", _nxtFootStep.pose.pos[0], _nxtFootStep.pose.pos[1], _nxtFootStep.pose.pos[2], zyx(0), zyx(1), zyx(2));
  printf("go %g %g %g, %g %g %g\n", _nxtFootStep.pose.pos[0], _nxtFootStep.pose.pos[1], _nxtFootStep.pose.pos[2], zyx(0), zyx(1), zyx(2));
  _takeStep = true; 
}
 
void Task2WalkingCon::gotoState(const RobotState &rs, Task2WalkingConState nxt)
{
  int old = _state;
  _state = nxt;
  _stateTime = 0;
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
    case HoldLeftFoot:
      _contactState = SSR;
      break;
    case LiftoffRight:
    case SwingUpRight:
    case SwingDownRight:
    case HoldRightFoot:
      _contactState = SSL;
      break;
  }

  fprintf(stderr, "T2C \"%s\" -> \"%s\" @ %g\n", conModeNames[old].c_str(), conModeNames[_state].c_str(), rs.time);
  printf("T2C \"%s\" -> \"%s\" @ %g\n", conModeNames[old].c_str(), conModeNames[_state].c_str(), rs.time);
}

int Task2WalkingCon::control(RobotState &rs, Command &cmd)
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
     
      if (_stateTime >= _ds_duration && _go != STOP) {
        if (_go == START_RIGHT) {
          // swapped by resetAfterStep
          _stanceFoot = RIGHT;
          _swingFoot = LEFT;
          _curHeading = (getYaw(rs.feet[LEFT].w_q)+getYaw(rs.feet[RIGHT].w_q)) / 2.;
          _nxtOffset[5] = 0;
          resetAfterStep(rs);

          setupComDs(rs, -1, LEFT); 
          gotoState(rs, ShiftComToLeft);
        }
        else if (_go == START_LEFT) {
          // swapped by resetAfterStep
          _stanceFoot = LEFT;
          _swingFoot = RIGHT;
          _curHeading = (getYaw(rs.feet[LEFT].w_q)+getYaw(rs.feet[RIGHT].w_q)) / 2.;
          _nxtOffset[5] = 0;
          resetAfterStep(rs);

          setupComDs(rs, -1, RIGHT); 
          gotoState(rs, ShiftComToRight);
        }
      }
      
      break;
   
    case HoldRightFoot:
      interpFoot(rs, foot_d[RIGHT]);
      _footStage[RIGHT] = WalkingCon::Swing;

      fillDesireds(rs, root_d, foot_d);
      
      if (_takeStep) {
        setupSwingDown(rs);
        gotoState(rs, SwingDownRight);
      }
      break; 

    case HoldLeftFoot:
      interpFoot(rs, foot_d[LEFT]);
      _footStage[LEFT] = WalkingCon::Swing;

      fillDesireds(rs, root_d, foot_d);
      
      if (_takeStep) {
        setupSwingDown(rs);
        gotoState(rs, SwingDownLeft);
      }
      
      break; 
  
    case FixCopLeft:
      fillDesireds(rs, root_d, foot_d);
      _footStage[RIGHT] = WalkingCon::LiftOff;

      if (fabs(w_cop2b_cop(rs,LEFT)[YY]) < 1e-2)
        good_cop_ctr++;
      else
        good_cop_ctr = 0;

      if (good_cop_ctr > 200 || _stateTime >= 10) {
        if (_stateTime >= 10)
          _com_int_flag[LEFT] = false;

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

      if (good_cop_ctr > 200 || _stateTime >= 10) {
        if (_stateTime >= 10)
          _com_int_flag[RIGHT] = false;

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
      if (_stateTime > _ds_duration - 0.2)
        _footStage[RIGHT] = WalkingCon::LiftOff;

      if (_stateTime >= _ds_duration) {
        _com_int_flag[LEFT] = true;
        gotoState(rs, FixCopLeft);
      }

      break;

    case ShiftComToRight:
      fillDesireds(rs, root_d, foot_d);

      if (_stateTime < 0.2 && _shiftCOMstart != -1)
        _footStage[RIGHT] = WalkingCon::TouchDown;
      if (_stateTime > _ds_duration - 0.2)
        _footStage[LEFT] = WalkingCon::LiftOff;

      if (_stateTime >= _ds_duration) {
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
      //if (_stateTime > 1.5*_ss_duration) 
      {
        ikCon->foot_q_weight = ikCon->IK_FOOT_WEIGHT;
        gotoState(rs, HoldLeftFoot);
      } 

      break;

    case SwingUpRight:
      interpFoot(rs, foot_d[RIGHT]);
      fillDesireds(rs, root_d, foot_d);
      _footStage[RIGHT] = WalkingCon::Swing;
      
      if (rs.time > _swgFoot1[0].time) 
      //if (_stateTime > 1.5*_ss_duration) 
      {
        ikCon->foot_q_weight = ikCon->IK_FOOT_WEIGHT;
        gotoState(rs, HoldRightFoot);
      }  

      break;

    case SwingDownLeft:
      interpFoot(rs, foot_d[LEFT]);
      if (_stateTime > _ss_duration - 0.5)
        _footStage[LEFT] = WalkingCon::TouchDown;
      else 
        _footStage[LEFT] = WalkingCon::Swing;

      fillDesireds(rs, root_d, foot_d);

      if (rs.feet[LEFT].b_F[ZZ] > touch_down_fz_thres)
      { 
        printf("left fz %g\n", rs.feet[LEFT].b_F[ZZ]);

        _lastFeet[LEFT] = Eigen::Map <const Eigen::Vector3d> (rs.feet[LEFT].w_sensor_pos); 

        _com_int_flag[RIGHT] = false;
        resetAfterStep(rs);
        if (_go != STOP) {
          setupComDs(rs, RIGHT, LEFT); 
          gotoState(rs, ShiftComToLeft);
        }
        else {
          setupComDs(rs, RIGHT, -1); 
          gotoState(rs, Idle);
        }
      }
      break;

    case SwingDownRight:
      interpFoot(rs, foot_d[RIGHT]);
      if (_stateTime > _ss_duration - 0.5)
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
        resetAfterStep(rs);
        if (_go != STOP) {
          setupComDs(rs, LEFT, RIGHT); 
          gotoState(rs, ShiftComToRight);
        }
        else {
          setupComDs(rs, LEFT, -1); 
          gotoState(rs, Idle);
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

  /*
  //////////////////////////////////////////////////////////////////////////////
  // integrate com offset towards cop mid
  for (int i = 0; i < 2; i++) {
    linear_spline(rs.time, _rs_com0+i, _rs_com1+i, ikcmd.com_offset+i);
    rs.com[i] += ikcmd.com_offset[i];
  }
  
  for (int side = 0; side < LR; side++) {
    if (_com_int_flag[side]) {
      //Eigen::Vector2d body_frame = w_cop2b_cop(rs, side);

      //_com_d_i[side][XX] += com_int_i[XX]*(_com_d_over_foot[XX] - rs.feet[side].b_cop[XX]);
      //_com_d_i[side][YY] += com_int_i[YY]*(_com_d_over_foot[YY] - rs.feet[side].b_cop[YY]);
      
      //_com_d_i[side][XX] += com_int_i[XX]*(_com_d_over_foot[XX] - body_frame[XX]);
      //_com_d_i[side][YY] += com_int_i[YY]*(_com_d_over_foot[YY] - body_frame[YY]);
      
      for (int i = 0; i < 2; i++) {
        _com_d_i[side][i] -= com_int_i[i]*(ikcmd.com[i] - rs.cop[i]);
        clamp(_com_d_i[side][i], -0.1, 0.1);
        _rs_com1[i].pos = _com_d_i[side][i];
      }
    }
  }
  */
  //////////////////////////////////////////////////////////////////////////////

  return 0;
}

Eigen::Vector2d Task2WalkingCon::w_cop2b_cop(const RobotState &rs, int side)
{
  Eigen::Vector2d w_cop(rs.cop[0]-rs.feet[side].w_sensor_pos[0], rs.cop[1]-rs.feet[side].w_sensor_pos[1]);
  return rotate2d(w_cop, -getYaw(rs.feet[side].w_q)); 
}

void Task2WalkingCon::interp(const RobotState &rs, TrajPoint <12,0> &com_d)
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

void Task2WalkingCon::interpFoot(const RobotState &rs, TrajPoint <7,0> &foot_d)
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

void Task2WalkingCon::setupSwingLiftoff(const RobotState &rs)
{
  double pos0[9] = {0}, pos1[9] = {0};
  double t0 = rs.time;
  double t1;
  
  double z_d = liftoff_height;
  double dz = _nxtFootStep.pose.pos[2] - _lastFeet[_swingFoot][ZZ];

  _lastZ = std::max(ikCon->ikrs->feet[_swingFoot].w_sensor_pos[ZZ], 
                    ikCon->ikrs->feet[_swingFoot].w_toe_pos[ZZ]);

  fprintf(stderr, "side %d, mode %s\n", _swingFoot, TerrainModeNames[_terrainMode].c_str());
  printf("side %d, mode %s\n", _swingFoot, TerrainModeNames[_terrainMode].c_str());
  
  if (_terrainMode == WalkingCon::Flat) {
    _ss_duration = ss_flat_duration;
    _ds_duration = ds_flat_duration;
  }
  else {
    _ss_duration = ss_block_duration;
    _ds_duration = ds_block_duration;
  }
  // pull it out from IK
  if (_doToeOff) {
    dvec_copy(pos0, ikCon->ikrs->feet[_swingFoot].w_toe_pos, 3);
    dvec_copy(pos1, ikCon->ikrs->feet[_swingFoot].w_toe_pos, 3);
  }
  else {
    dvec_copy(pos0, ikCon->ikrs->feet[_swingFoot].w_sensor_pos, 3);
    dvec_copy(pos1, ikCon->ikrs->feet[_swingFoot].w_sensor_pos, 3);
  }

  if (_terrainMode == WalkingCon::UpBlock) {
    z_d += dz;
  }  
  
  _liftoffDuration = std::max(1.5, fabs(z_d) / liftoff_rate);
  t1 = t0 + _liftoffDuration; 

  // joint control for this phase
  for (int i = 0; i < 6; i++) {
    double j[2]; 
    int idx = A_L_LEG_UHZ+i+6*_swingFoot;
    j[0] = j[1] = ikCon->ikrs->joints[idx];
    // hip
    if (i == 2) {
      if (_terrainMode == WalkingCon::DownBlock)
        j[1] -= 0.4;
      else
        j[1] -= 0.2;
    }
    // knee
    if (i == 3) {
      if (_terrainMode == WalkingCon::UpBlock)
        j[1] = 1.6;
      else
        j[1] += 0.5;
    }
    // ankle 
    if (i == 4)
      j[1] = -0.9;

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

void Task2WalkingCon::setupSwingUp(const RobotState &rs)
{
  double pos0[9] = {0}, pos1[9] = {0};
  double t0 = rs.time;
  double t1 = t0;
  //double t1 = t0 + 1*_ss_duration;
  double slerpt; 

  _swingLegJointMode = false;

  for (int i = 0; i < 3; i++)
    pos0[i] = ikCon->ikrs->feet[_swingFoot].w_sensor_pos[i];
  dvec_copy(pos1, _nxtFootStep.pose.pos.data(), 3);
  pos1[2] += ss_height;

  // max between liftoff end height, and target hover height
  pos1[2] = std::max(pos0[2], pos1[2]);
  // for the last segment, raise the swing up more for the flat block
  // and the first swing up when going up.
  if (_scriptType == TiltetPyramid && 
      (_terrainMode == Flat || (_terrainMode == UpBlock && _upCtr == 1)))
    pos1[2] += 0.04;
  
  Eigen::Vector3d diff = Eigen::Map<Eigen::Vector3d>(pos0) - Eigen::Map<Eigen::Vector3d>(pos1);
  double dt = diff.norm() / swing_rate;
  dt = std::max(1., dt);
  if (_terrainMode == DownBlock && _downCtr == 1)
    dt *= 1.5;
  t1 += dt;

  _swgQ0 = ikCon->ikrs->feet[_swingFoot].w_q;
  _swgQ1 = _nxtFootStep.pose.rot;

  printf("swg up, pos0 %g %g %g, pos1 %g %g %g\n", pos0[0], pos0[1], pos0[2], pos1[0], pos1[1], pos1[2]);

  set_knots_pos_only(t0, t1, pos0, pos1, pos0+3, pos1+3, pos0+6, pos1+6, _swgFoot0, _swgFoot1);
}
 
void Task2WalkingCon::setupSwingDown(const RobotState &rs)
{
  for (int i = 0; i < 3; i++) {
    _swgFoot0[i].pos = _swgFoot1[i].pos;
    _swgFoot0[i].time = rs.time;
    _swgFoot1[i].pos = _nxtFootStep.pose.pos[i];
    _swgFoot1[i].time = rs.time + 0.5*_ss_duration;

    _swgFoot0[i].acc = 0;
    _swgFoot0[i].vel = 0;
    _swgFoot1[i].acc = 0;
    _swgFoot1[i].vel = 0;
  }
  _swgFoot1[2].pos += 0.02;
  _swgQ0 = ikCon->ikrs->feet[_swingFoot].w_q; //rs.feet[RIGHT].w_q;
  _swgQ1 = _nxtFootStep.pose.rot;
}

Eigen::Vector3d Task2WalkingCon::computeCom_d(const RobotState &rs, int side, double dz)
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
    
    if (_terrainMode == WalkingCon::UpBlock) {
      dx[0] += 0.01;
      _com_d_over_foot[XX] = Foot::ankle_to_center_offset[XX]+0.01;
      _com_d_over_foot[YY] = Foot::ankle_to_center_offset[YY];
    }
    if (_terrainMode == WalkingCon::DownBlock) {
      dx[0] += 0.0;
      _com_d_over_foot[XX] = Foot::ankle_to_center_offset[XX]+0.0;
      _com_d_over_foot[YY] = Foot::ankle_to_center_offset[YY];
    }
    if (_terrainMode == WalkingCon::UpSlope) {
      dx[0] += 0.05;
      _com_d_over_foot[XX] = Foot::ankle_to_center_offset[XX]+0.05;
      _com_d_over_foot[YY] = Foot::ankle_to_center_offset[YY];
    }
    if (_terrainMode == WalkingCon::DownSlope) {
      _com_d_over_foot[XX] = Foot::ankle_to_center_offset[XX];
      _com_d_over_foot[YY] = Foot::ankle_to_center_offset[YY];
    }

    double yaw = getYaw(rs.feet[side].w_q);
    double offset[3] = {cos(yaw)*dx[0] - sin(yaw)*dx[1], sin(yaw)*dx[0] + cos(yaw)*dx[1], 0};

    for (int i = 0; i < 3; i++)
      ret[i] = rs.feet[side].w_mid_pos[i] + offset[i];
  }
  
  // fix z
  ret[ZZ] += com_z;

  // some hacks about z
  if (_contactState == DSc) {
    /*
    // shift weight to the foot right before block
    if (_terrainMode == WalkingCon::UpBlock && _upCtr == 1) 
      ret[ZZ] = _com0[ZZ].pos + 0.05; 
    // shift weight to the foot ON the block
    if (_terrainMode == WalkingCon::UpBlock && _upCtr == 2) 
      ret[ZZ] = _com0[ZZ].pos + 0.02;
    */
    if (_terrainMode == WalkingCon::UpBlock) 
      ret[ZZ] = _com0[ZZ].pos + 0.03;
    // shift com to the foot close to edge
    if (_terrainMode == WalkingCon::DownBlock && _downCtr == 1) 
      ret[ZZ] -= 0.08;
    // shift com down the block
    if (_terrainMode == WalkingCon::DownBlock && _downCtr == 2) 
      ret[ZZ] = _com0[ZZ].pos + 0.07;

    /*
    if (_terrainMode == WalkingCon::UpSlope)
      ret[ZZ] = _com0[ZZ].pos;
    
    if (_terrainMode == WalkingCon::DownSlope)
      ret[ZZ] = _com0[ZZ].pos;
    */
  }
  // swing 
  else {
    /*
    // lift lead foot up to the block
    if (_terrainMode == WalkingCon::UpBlock && _upCtr == 1) 
      ret[ZZ] += 0.08;
    // lift rear foot up to the block
    if (_terrainMode == WalkingCon::UpBlock && _upCtr == 2) 
      ret[ZZ] += 0.05;
    */
    if (_terrainMode == WalkingCon::UpBlock) 
      ret[ZZ] += 0.05;

    if (_terrainMode == WalkingCon::Flat) { }

    // move lead foot down the block
    if (_terrainMode == WalkingCon::DownBlock && _downCtr == 1) 
      ret[ZZ] -= 0.13;
    // move rear foot down the block
    if (_terrainMode == WalkingCon::DownBlock && _downCtr == 2) 
      ret[ZZ] = _com0[ZZ].pos;
  }

  return ret;
}

void Task2WalkingCon::updateKnot(const RobotState &rs)
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

  // lower the last couple cm
  static double dz_hack = 0;

  if (_state == SwingUpLeft || _state == SwingUpRight || 
      _state == HoldLeftFoot || _state == HoldRightFoot)
  {
    double swing_lhy = rs.joints[A_L_LEG_LHY+6*_swingFoot];
    bool high_enough = false;
    if (_terrainMode != DownBlock && _swgFoot1[2].pos > _nxtFootStep.pose.pos[2] + ss_height)
      high_enough = true;
    if (_terrainMode == DownBlock && _swgFoot1[2].pos > _lastZ + 0.05)
      high_enough = true;

    if (swing_lhy < -1.5 && high_enough) 
      _swgFoot1[2].pos += 3*ss_lower_z_vel * rs.timeStep;
  }

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

    // lower pelvis when swing knee goes straight
    if (rs.joints[A_L_LEG_KNY] < 0.6) {
      _com1[2].pos += ss_lower_z_vel * rs.timeStep;
      _com1[2].vel = ss_lower_z_vel;
    }
    else 
      _com1[2].vel = 0;
  }
  else if (_state == SwingDownRight) {
    if (rs.time > _swgFoot1[2].time) {
      dz_hack += ss_lower_z_vel * rs.timeStep;
      _swgFoot1[2].vel = ss_lower_z_vel;
      _com1[2].pos += ss_lower_z_vel * rs.timeStep;
      _com1[2].vel = ss_lower_z_vel;
    }
    else {
      dz_hack = 0;
      _swgFoot1[2].vel = 0;
    }
    _swgFoot1[2].pos = _nxtFootStep.pose.pos[2] + 0.02 + dz_hack; 
    
    // lower pelvis when swing knee goes straight
    if (rs.joints[A_R_LEG_KNY] < 0.6) {
      _com1[2].pos += ss_lower_z_vel * rs.timeStep;
      _com1[2].vel = ss_lower_z_vel;
    }
    else 
      _com1[2].vel = 0;
  }
}

bool Task2WalkingCon::setupComSs(const RobotState &rs)
{
  double pos0[9] = {0}, pos1[9] = {0};
  double t0 = rs.time;
  double t1 = t0 + _ss_duration + _liftoffDuration;
  double slerpt;
  
  // setup orientation knots
  _pelvQ0 = _pelvQ1;
  _torsoQ0 = _torsoQ1;

  _torsoQ1 = zyx2quat(Eigen::Vector3d(0, 0.3, getYaw(rs.feet[_stanceFoot].w_q)));
  _pelvQ1 = zyx2quat(Eigen::Vector3d(0, 0.2, getYaw(rs.feet[_stanceFoot].w_q)));

  /*
  if (_terrainMode == WalkingCon::UpSlope) {
    _torsoQ1 = zyx2quat(Eigen::Vector3d(0, 0.4, getYaw(rs.feet[_stanceFoot].w_q)));
    _pelvQ1 = zyx2quat(Eigen::Vector3d(0, 0.3, getYaw(rs.feet[_stanceFoot].w_q)));
  }
  */

  /*
  if (_terrainMode == WalkingCon::UpBlock) {
    _torsoQ1 = zyx2quat(Eigen::Vector3d(0, 0.3, getYaw(rs.feet[_stanceFoot].w_q)));
    _pelvQ1 = zyx2quat(Eigen::Vector3d(0, 0.2, getYaw(rs.feet[_stanceFoot].w_q)));
  }
  else {
    _torsoQ1 = zyx2quat(Eigen::Vector3d(0, 0., getYaw(rs.feet[_stanceFoot].w_q)));
    _pelvQ1 = zyx2quat(Eigen::Vector3d(0, 0., getYaw(rs.feet[_stanceFoot].w_q)));
  }
  */

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

bool Task2WalkingCon::setupComDs(const RobotState &rs, int start, int end)
{
  double pos0[9] = {0}, pos1[9] = {0};
  double t0 = rs.time;
  double t1 = t0 + _ds_duration;
  double slerpt;

  double wl0, wl1;
 
  _shiftCOMstart = start;
  _shiftCOMend = end;
  
  // setup orientation knots
  _pelvQ0 = _pelvQ1;
  _torsoQ0 = _torsoQ1;

  double foot_yaw[2] = {getYaw(rs.feet[LEFT].w_q), getYaw(rs.feet[RIGHT].w_q)};
  //double yaw_avg = (foot_yaw[0] + foot_yaw[1]) / 2.;
  double yaw_avg = avgAngle(foot_yaw[0], foot_yaw[1]);

  _torsoQ1 = zyx2quat(Eigen::Vector3d(0, 0.3, yaw_avg));
  _pelvQ1 = zyx2quat(Eigen::Vector3d(0, 0.2, yaw_avg));

  /*
  // hack for going down, com offset
  //if (_prevTerrainMode == WalkingCon::DownBlock) {
  if ((_terrainMode == WalkingCon::DownBlock && _downCtr == 1 && _prevTerrainMode == WalkingCon::DownBlock) || 
      (_terrainMode != WalkingCon::DownBlock && _prevTerrainMode == WalkingCon::DownBlock)) 
  {
    if (end == RIGHT)
      _com_d_i[RIGHT][YY] += 0.03;
    if (end == LEFT)
      _com_d_i[LEFT][YY] -= 0.02;
  }
  */

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

  // rs com offset knots
  for (int i = 0; i < 2; i++)
    pos0[i] = _rs_com1[i].pos;

  if (end == LEFT) {
    pos1[0] = _com_d_i[LEFT][0];
    pos1[1] = _com_d_i[LEFT][1];
  }
  else if (end == RIGHT) {
    pos1[0] = _com_d_i[RIGHT][0];
    pos1[1] = _com_d_i[RIGHT][1];
  }
  else {
    pos1[0] = 0;
    pos1[1] = 0;
  }

  for (int i = 0; i < 2; i++)
    set_single_knot(t0, t0+2, pos0[i], pos1[i], _rs_com0+i, _rs_com1+i);

  return true;
}

void Task2WalkingCon::fillDesireds(const RobotState &rs, const TrajPoint<12,0> &root_d, const TrajPoint<7,0> foot_d[2])
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

void Task2WalkingCon::buildLookup()
{
  std::string name;

  _conName = "T2C";

  _lookup["fd_joint_p"] = &fd_joint_p;
  _lookup["fd_joint_d"] = &fd_joint_d;

  _lookup["com_z"] = &com_z;
  _lookup["ds_flat_duration"] = &ds_flat_duration;
  _lookup["ds_block_duration"] = &ds_block_duration;
  _lookup["ss_flat_duration"] = &ss_flat_duration;
  _lookup["ss_block_duration"] = &ss_block_duration;
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

const std::string Task2WalkingCon::scriptModeNames[NUM_SCRIPT_MODE] = 
{
  "Man",
  "Ramp",
  "ZigZag",
  "FlatTop",
  "Tilted",
  "InPlace",
  "BigStride"
};


const std::string Task2WalkingCon::conModeNames[NUM_CON_STATE] = 
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
  "HoldLeftFoot",
  "HoldRightFoot",
  "Idle"
};
   
void Task2WalkingCon::addToLog(BatchLogger &logger)
{
  logger.add_datapoint("T2C.state", "-", &(_state));
  logger.add_datapoint("T2C.terrainMode", "-", (const int *)&(_terrainMode));
  logger.add_datapoint("T2C.footStage[L]", "-", (const int *)&(_footStage[LEFT]));
  logger.add_datapoint("T2C.footStage[R]", "-", (const int *)&(_footStage[RIGHT]));
  logger.add_datapoint("T2C.upCtr", "-", &(_upCtr));
  logger.add_datapoint("T2C.downCtr", "-", &(_downCtr));
  logger.add_datapoint("T2C._cop_comp_z", "-", &_cop_comp_z);
  logger.add_datapoint("T2C.com_d_i[L][X]", "m", &(_com_d_i[LEFT][XX]));
  logger.add_datapoint("T2C.com_d_i[L][Y]", "m", &(_com_d_i[LEFT][YY]));
  logger.add_datapoint("T2C.com_d_i[R][X]", "m", &(_com_d_i[RIGHT][XX]));
  logger.add_datapoint("T2C.com_d_i[R][Y]", "m", &(_com_d_i[RIGHT][YY]));
  logger.add_datapoint("T2C.foot_d_i[L][X]", "m", &(_foot_d_i[LEFT][XX]));
  logger.add_datapoint("T2C.foot_d_i[L][Y]", "m", &(_foot_d_i[LEFT][YY]));
  logger.add_datapoint("T2C.foot_d_i[R][X]", "m", &(_foot_d_i[RIGHT][XX]));
  logger.add_datapoint("T2C.foot_d_i[R][Y]", "m", &(_foot_d_i[RIGHT][YY]));
  
  logger.add_datapoint("T2C.swinglegjointMode", "-", &(_swingLegJointMode));

  _nxtFootStep.addToLog(logger);

  logger.add_datapoint("T2C.com0[X]", "m", &(_com0[XX].pos));
  logger.add_datapoint("T2C.com0[Y]", "m", &(_com0[YY].pos));
  logger.add_datapoint("T2C.com0[Z]", "m", &(_com0[ZZ].pos));
  logger.add_datapoint("T2C.com1[X]", "m", &(_com1[XX].pos));
  logger.add_datapoint("T2C.com1[Y]", "m", &(_com1[YY].pos));
  logger.add_datapoint("T2C.com1[Z]", "m", &(_com1[ZZ].pos));
  
  logger.add_datapoint("T2C.swgFoot0[X]", "m", &(_swgFoot0[XX].pos));
  logger.add_datapoint("T2C.swgFoot0[Y]", "m", &(_swgFoot0[YY].pos));
  logger.add_datapoint("T2C.swgFoot0[Z]", "m", &(_swgFoot0[ZZ].pos));
  logger.add_datapoint("T2C.swgFoot1[X]", "m", &(_swgFoot1[XX].pos));
  logger.add_datapoint("T2C.swgFoot1[Y]", "m", &(_swgFoot1[YY].pos));
  logger.add_datapoint("T2C.swgFoot1[Z]", "m", &(_swgFoot1[ZZ].pos));
  
  logger.add_datapoint("T2C.rf[L][X]", "m", &(_realFoot_d[LEFT][XX]));
  logger.add_datapoint("T2C.rf[L][Y]", "m", &(_realFoot_d[LEFT][YY]));
  logger.add_datapoint("T2C.rf[L][Z]", "m", &(_realFoot_d[LEFT][ZZ]));
  logger.add_datapoint("T2C.rf[R][X]", "m", &(_realFoot_d[RIGHT][XX]));
  logger.add_datapoint("T2C.rf[R][Y]", "m", &(_realFoot_d[RIGHT][YY]));
  logger.add_datapoint("T2C.rf[R][Z]", "m", &(_realFoot_d[RIGHT][ZZ]));
}  

void Task2WalkingCon::makeScript(Task2ScriptMode part)
{
  _script.clear();
  _scriptType = part;
  
  // second col is swing side
  switch(part) {
    case Ramp:
      printf("ramp script\n");
      fprintf(stderr, "ramp script\n");
      /*
      // right before ramp
      _script.push_back(UpSlope);     // r
      
      // up ramp steps
      _script.push_back(UpSlope);     // l
      _script.push_back(UpSlope);     // r
      _script.push_back(UpSlope);     // l
      _script.push_back(DownSlope);   // r
      
      // down ramp steps
      _script.push_back(DownSlope);   // l
      _script.push_back(DownSlope);   // r
      _script.push_back(DownSlope);   // l
      _script.push_back(Flat);        // r
      
      // on ground
      _script.push_back(Flat);        // l
      */
      
      // right before ramp
      _script.push_back(ScriptData(UpSlope, 0.3));     // r
      
      // up ramp steps
      _script.push_back(ScriptData(UpSlope, 0.18));     // l
      _script.push_back(ScriptData(UpSlope, 0.18));     // r
      _script.push_back(ScriptData(DownSlope, 0.3));   // l
      
      // down ramp steps
      _script.push_back(ScriptData(DownSlope, 0.18));   // r
      _script.push_back(ScriptData(DownSlope, 0.18));   // l
      _script.push_back(ScriptData(Flat, 0.26));        // r
      
      // on ground
      _script.push_back(ScriptData(Flat, 0.1));        // l 
      break;
    
    case ZigZag:
      fprintf(stderr, "zigzag script\n");
      // right before zigzag
      _script.push_back(ScriptData(UpBlock, 0.2));     // r

      // zigzag
      _script.push_back(ScriptData(UpBlock, 0.2));     // l  
      _script.push_back(ScriptData(DownBlock, 0.15));   // r

      // on ground
      _script.push_back(ScriptData(DownBlock, 0.1));   // l
      //_script.push_back(Flat);        // r
      break;

    case FlatPyramid:
      printf("flat top script\n");
      fprintf(stderr, "flat top script\n");
      // right before flat top 
      _script.push_back(ScriptData(UpBlock, 0.3));     // r

      // first block
      _script.push_back(ScriptData(UpBlock, 0.1));     // l
      _script.push_back(ScriptData(UpBlock, 0.3));     // r

      // second block
      _script.push_back(ScriptData(UpBlock, 0.1));     // l
      _script.push_back(ScriptData(UpBlock, 0.3));     // r

      // thrid block
      _script.push_back(ScriptData(UpBlock, 0.1));     // l
      _script.push_back(ScriptData(UpBlock, 0.3));     // r
      
      // fourth block, top
      _script.push_back(ScriptData(UpBlock, 0.1));     // l
      _script.push_back(ScriptData(DownBlock, 0.3));   // r

      // fifth
      _script.push_back(ScriptData(DownBlock, 0.1));   // l
      _script.push_back(ScriptData(DownBlock, 0.3));   // r
      
      // sixth
      _script.push_back(ScriptData(DownBlock, 0.1));   // l
      _script.push_back(ScriptData(DownBlock, 0.3));   // r
      
      // seventh
      _script.push_back(ScriptData(DownBlock, 0.1));   // l
      _script.push_back(ScriptData(DownBlock, 0.3));   // r
      
      // on ground
      _script.push_back(ScriptData(DownBlock, 0.1));   // l
      //_script.push_back(Flat);        // r
      break;

    case TiltetPyramid:
      printf("tilted script\n");
      fprintf(stderr, "tilted script\n");
      // right before
      _script.push_back(ScriptData(UpBlock, 0.3));     // r

      // first flat top
      _script.push_back(ScriptData(UpBlock, 0.1));     // l
      _script.push_back(ScriptData(Flat, 0.3));        // r
      
      // up, right 
      _script.push_back(ScriptData(Flat, 0.1));        // l
      _script.push_back(ScriptData(UpBlock, 0.3));     // r

      // up, down
      _script.push_back(ScriptData(UpBlock, 0.1));     // l
      _script.push_back(ScriptData(UpBlock, 0.3));     // r

      // up, right
      _script.push_back(ScriptData(UpBlock, 0.1));     // l
      _script.push_back(ScriptData(UpBlock, 0.3));     // r

      // top, down
      _script.push_back(ScriptData(UpBlock, 0.1));     // l
      _script.push_back(ScriptData(Flat, 0));        // r
      _script.push_back(ScriptData(DownBlock, 0.3));   // l

      // down, right
      _script.push_back(ScriptData(DownBlock, 0.1));   // r
      _script.push_back(ScriptData(DownBlock, 0.3));   // l

      // down, down
      _script.push_back(ScriptData(DownBlock, 0.1));   // r
      _script.push_back(ScriptData(DownBlock, 0.3));   // l
      
      // down, right
      _script.push_back(ScriptData(DownBlock, 0.1));   // r
      _script.push_back(ScriptData(DownBlock, 0.3));   // l

      // on ground
      _script.push_back(ScriptData(DownBlock, 0.1));   // r
      //_script.push_back(Flat);        // l
      break;

    case StepInPlace:
      printf("step in place script\n");
      fprintf(stderr, "step in place script\n");
      for (int i = 0; i < 100; i++)
        _script.push_back(ScriptData(Flat, 0.));
      break;
 
    case BigStride:
      printf("big stride script\n");
      fprintf(stderr, "big stride place script\n");
      for (int i = 0; i < 100; i++)
        _script.push_back(ScriptData(Flat, 0.3));
      break; 

    default:
      _scriptType = Manual;
      _go = STOP;
      printf("manual drive\n");
      fprintf(stderr, "manual drive\n");
      break;
  }
}

bool Task2WalkingCon::isHalt() const
{
  return _go == STOP;  
}

