/*
 * =====================================================================================
 *
 *       Filename:  StaticWalkingCon.cpp
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

#include "StaticWalkingCon.h"
#include "Eigen_utils.hpp"
#include "spline.h"
#include <map>
#include <boost/thread.hpp>

const std::string StaticWalkingCon::conModeNames[NUM_CON_STATE] = 
{
  "Com to Left",
  "Com to Right",
  "Left lift off",
  "Left swing up",
  "Left swing down",
  "Left touch down",
  "Right lift off",
  "Right swing up",
  "Right swing down",
  "Right touch down",
  "Idle"
};

enum StoppingStages {
  NoStop = 0,
  StopSignaled,
  LastSwing,
  Stopped
};

StaticWalkingCon::StaticWalkingCon()
{
  buildLookup();

  step_length = 0.15;
  step_width = 0.14;
  com_z = 0.81;
  ds_duration = 3;
  ss_duration = 2;
  ss_height = 0.1;
  fd_joint_p = 100;
  fd_joint_d = 10;
  ss_lower_z_rate = 0.008;

  up_foot_pitch = -0.4;
  up_torso_pitch = 0.3;
  up_ss_height = 0.2;
  up_com_z = 0.8;
  up_step_length = 0.15;

  for (int i = 0; i < 3; i++) {
    fd_com_p[i] = 100;
    fd_com_d[i] = 10;
    fd_com_p[i+3] = 150;
    fd_com_d[i+3] = 10;

    fd_foot_p[i] = 150;
    fd_foot_d[i] = 10;
    fd_foot_i[i] = 0.1;

    fd_foot_p[i+3] = 150;
    fd_foot_d[i+3] = 10;
    fd_foot_i[i+3] = 0.1;
  }

  stance_lax_d = 10;
  stance_uay_d = 10;
  stance_uhz_d = 10;
  touch_down_fz_thres = 0.02;
  lift_off_fz_thres = 0.15;
  w_high_thres = 0.8;
  w_low_thres = 0.2;

  _mudMode = false;
  _terrainMode = Flat;

  _idle = true;
  _stopping = NoStop;
  _inited = false;
}

StaticWalkingCon::~StaticWalkingCon()
{

}

void StaticWalkingCon::init(const RobotState &rs)
{
  _stateTime = 0;
  _stepCount = 0;
  _idle = true;
  _stopping = NoStop;
  _heading = rs.root[5];
  ikCon->setToRobotState(rs);
  gotoState(Idle);
  setupDs(rs, -1, -1); 
  _inited = true;

  _neck_ay_d = 1;

  printParams();
}

void StaticWalkingCon::switchToMudMode() 
{
  if (_mudMode)
    return;
  printf("turning integral control on... \n");
  _mudMode = true;
  //getchar();
}

void StaticWalkingCon::gotoState(StaticWalkingConState nxt)
{
  int old = _state;
  _state = nxt;
  _stateTime = 0;
  switch (nxt) {
    case LiftoffLeft:
    case LiftoffRight:
    case Idle:
      _contactState = DSc;
      break;
    case ShiftComToRight:
      _contactState = DSc;
      break;
    case TouchDownRight:
      _contactState = DSr;
      break;
    case ShiftComToLeft:
      _contactState = DSc;
      break;
    case TouchDownLeft:
      _contactState = DSl;
      break;
    case LiftLeftFoot:
    case LowerLeftFoot:
      _contactState = SSR;
      break;
    case LiftRightFoot:
    case LowerRightFoot:
      _contactState = SSL;
      break;
  }

  printf("SWC \"%s\" -> \"%s\"\n", conModeNames[old].c_str(), conModeNames[_state].c_str());
  _swingFeetPosIntegral[LEFT] = Eigen::Vector3d::Zero();
  _swingFeetRotIntegral[LEFT] = Eigen::Vector3d::Zero();
  _swingFeetPosIntegral[RIGHT] = Eigen::Vector3d::Zero();
  _swingFeetRotIntegral[RIGHT] = Eigen::Vector3d::Zero();
}

Pose StaticWalkingCon::makeNxtStraightFootStep(const RobotState &rs, int side)
{
  Pose nxt;
  double stance[2] = {0};
  double ang = 0;
  double dx, dz;

  if (side == LEFT) {
    nxt.pos = _curFeet[RIGHT].pos;
    ang = _heading + M_PI/2.;
  }
  else {
    nxt.pos = _curFeet[LEFT].pos;
    ang = _heading - M_PI/2.;
  }

  switch (_terrainMode) {
    case Flat:
      dx = step_length;
      dz = ss_height;
      break;
    case UpHill:
      dx = up_step_length;
      dz = up_ss_height;
      break;
    case DownHill:
      dx = up_step_length;
      dz = ss_height;
      break;
  }

  if (_stopping == StopSignaled) {
    dx = 0.;
    printf("SWC: making last swing before STOP. \n");
    _stopping = LastSwing;
  }

  if (_distTraveled + dx > _maxDist) {
    dx = _maxDist - _distTraveled;
    _stopping = StopSignaled;
  }

  // increment dist
  _distTraveled += dx;

  nxt.rot = zyx2quat(Eigen::Vector3d(0,0,_heading));
  nxt.pos[XX] += cos(_heading) * dx + cos(ang) * 2 * step_width;
  nxt.pos[YY] += sin(_heading) * dx + sin(ang) * 2 * step_width;
  nxt.pos[ZZ] += dz;

  return nxt;
}

/*
   void StaticWalkingCon::updateComTraj(const Eigen::Vector3d com, const Eigen::Vector3d &comd, const Eigen::Vector3d &foot, size_t idx)
   {
   knot_t k0[3];
   knot_t k1[3];
   double pos0[9] = {0}, pos1[9] = {0};
   double t0 = rs.time;
   double t1 = t0 + ds_duration;
   double dt = rs.timeStep;
   double slerpt;

// setup orientation knots
Eigen::Quaterniond root_q0 = rs.rootq;
Eigen::Quaterniond root_q1; 
// setup pos knots
dvec_copy(pos0, rs.com, 3);
dvec_copy(pos0+3, rs.comd, 3); 
}
*/

int StaticWalkingCon::control(RobotState &rs, Command &cmd)
{
  size_t rootIdx = _dsComTraj.getIdx(rs.time);
  size_t swingIdx;
  const double e = 0.9;

  _stateTime += rs.timeStep;

  for (int i = 0; i < 2; i++) {
    _curFeet[i] = Pose(Eigen::Map<const Eigen::Vector3d> (rs.feet[i].w_sensor_pos), rs.feet[i].w_q);
    /*
       if (!isSwing(rs.contactState, i)) {
       _stanceFeet[i].pos = e*_stanceFeet[i].pos + (1-e)*_curFeet[i].pos;
       _stanceFeet[i].rot = _stanceFeet[i].rot.slerp(1-e, _curFeet[i].rot).normalized();
       }
       */
  }

  Eigen::Vector3d feet_vel[LR] = {
    Eigen::Map<const Eigen::Vector3d> (rs.feet[LEFT].w_sensor_vel), 
    Eigen::Map<const Eigen::Vector3d> (rs.feet[RIGHT].w_sensor_vel)};
  Eigen::Vector3d feet_w[LR] = {
    Eigen::Map<const Eigen::Vector3d> (rs.feet[LEFT].w_sensor_vel+3), 
    Eigen::Map<const Eigen::Vector3d> (rs.feet[RIGHT].w_sensor_vel+3)};


  Eigen::Vector3d feet_b_w[LR] = {
    Eigen::Map<const Eigen::Vector3d> (rs.feet[LEFT].b_w), 
    Eigen::Map<const Eigen::Vector3d> (rs.feet[RIGHT].b_w)};

  static double temp_feet_b_w[2] = {0};

  double tmp[8] = {0}, tmpv[8] = {0};
  Eigen::Map<Eigen::Vector3d> tmpPos(tmp);
  Eigen::Map<Eigen::Vector3d> tmpVel(tmpv);
  Eigen::Map<Eigen::Quaterniond> tmpQ(tmp+3);
  double t;
  double t1;
  Eigen::Map<const Eigen::Vector3d> rootd(rs.rootd);
  Eigen::Map<const Eigen::Vector3d> rootw(rs.rootd+3);
  Eigen::Vector3d zyx;

  TrajPoint <8,0> root_d = _dsComTraj[rootIdx];
  TrajPoint <7,0> foot_d;

  static double lift_start_time = 0;
  double lift_end_time = _dsComTraj[_dsComTraj.size()-1].time;

  bool angSame;
  Eigen::Quaterniond negQ;

  switch (_state) {
    case Idle:
      fillDesireds(rs, root_d, foot_d);

      if (!_idle) {
        setupDs(rs, -1, LEFT); 
        _stateTime = 0;
        _stepCount = 0; 

        gotoState(ShiftComToLeft);
      }
      break;

    case ShiftComToLeft:
      //printf("to left w_l %g\n", root_d.pos[7]);
      fillDesireds(rs, root_d, foot_d);

      zyx = quat2zyx(_curFeet[LEFT].rot);

      //if(zyx[0]<-4/180*M_PI)
       // lift_off_fz_thres = 0.25;
      //idcmd.b_cop[RIGHT][XX] = temp_feet_b_w[0];
      //idcmd.b_cop[RIGHT][YY] = temp_feet_b_w[1];

      if ((_stateTime > ds_duration/2 && rs.feet[RIGHT].b_F[ZZ] < rs.m * GRAVITY * lift_off_fz_thres&& rs.feet[LEFT].b_F[ZZ] > rs.m * GRAVITY * (1-lift_off_fz_thres)) || 
          (rootIdx >= _dsComTraj.size()-1)) 
      {

        temp_feet_b_w[0]= 0;
        temp_feet_b_w[1]= 0;

        _stanceFeet[RIGHT] = _curFeet[RIGHT];
        _nxtFootStep = makeNxtStraightFootStep(rs, RIGHT);

        printf("f1 %g %g %g\n", _nxtFootStep.pos[XX], _nxtFootStep.pos[YY], _nxtFootStep.pos[ZZ]);
        lift_start_time = rs.time;
        gotoState(LiftRightFoot);
        //gotoState(LiftoffRight);
      }
      break;

    case ShiftComToRight:
      //printf("to right w_l %g\n", root_d.pos[7]);
      fillDesireds(rs, root_d, foot_d);

      zyx = quat2zyx(_curFeet[RIGHT].rot);

      //if(zyx[0]>4/180*M_PI)
       // lift_off_fz_thres = 0.25;
      //idcmd.b_cop[LEFT][XX] = temp_feet_b_w[0];
      //idcmd.b_cop[LEFT][YY] = temp_feet_b_w[1];

      if ((_stateTime > ds_duration/2 && rs.feet[LEFT].b_F[ZZ] < rs.m * GRAVITY * lift_off_fz_thres && rs.feet[RIGHT].b_F[ZZ] > rs.m * GRAVITY * (1-lift_off_fz_thres)) || 
          (rootIdx >= _dsComTraj.size()-1)) 
      {

        temp_feet_b_w[0]= 0;
        temp_feet_b_w[1]= 0;

        _stanceFeet[LEFT] = _curFeet[LEFT];
        _nxtFootStep = makeNxtStraightFootStep(rs, LEFT);

        printf("f1 %g %g %g\n", _nxtFootStep.pos[XX], _nxtFootStep.pos[YY], _nxtFootStep.pos[ZZ]); 
        lift_start_time = rs.time;
        gotoState(LiftLeftFoot);
        //gotoState(LiftoffLeft);
      }
      break;

    case LiftLeftFoot:
      t = _stateTime / ss_duration;
      t = std::min(t, 1.);
      t = std::max(t, 0.);

      t1 = (_stateTime - 0.2*ss_duration) / (0.8*ss_duration);
      t1 = std::min(t1, 1.);
      t1 = std::max(t1, 0.);

      zyx = quat2zyx(_curFeet[RIGHT].rot);
      //tmpPos = _stanceFeet[LEFT].pos*(1-t) + t*_nxtFootStep.pos;

      //tmpPos = _stanceFeet[LEFT].pos + (1-cos(2*M_PI*t1))/2*(_nxtFootStep.pos-_stanceFeet[LEFT].pos);
      //tmpPos[2] = _stanceFeet[LEFT].pos[2] + (1-cos(2*M_PI*t))/2*(_nxtFootStep.pos[2] - _stanceFeet[LEFT].pos[2]);

      tmpPos = _stanceFeet[LEFT].pos + t1*(_nxtFootStep.pos-_stanceFeet[LEFT].pos);
      tmpPos[2] = _stanceFeet[LEFT].pos[2] + t*(_nxtFootStep.pos[2] - _stanceFeet[LEFT].pos[2]);

      tmpVel = Eigen::Vector3d::Zero();

      if(t1 == 0) {
        tmpQ = _stanceFeet[LEFT].rot; //_curFeet[LEFT].rot;
        tmpVel[2] = 0.1;
      }
      else
        //tmpQ = zyx2quat(Eigen::Vector3d(0,zyx[1],_heading));
        tmpQ = _stanceFeet[LEFT].rot.slerp(t, zyx2quat(Eigen::Vector3d(0,zyx[1],_heading)));

      foot_d = TrajPoint<7,0>(rs.time, rs.contactState, tmp, tmpv, NULL, NULL);

      t = (rs.time-lift_start_time) / (lift_end_time-lift_start_time);
      t = std::min(t, 1.);
      t = std::max(t, 0.);

      tmpPos = (1-t)*Eigen::Map<const Eigen::Vector3d>(root_d.pos) + t*(_curFeet[RIGHT].pos+Eigen::Vector3d(0,0,com_z));
      tmpVel = Eigen::Vector3d::Zero();
      tmpQ = Eigen::Map<const Eigen::Quaterniond>(_dsComTraj[rootIdx].pos+3);
      root_d = TrajPoint<8,0>(rs.time, rs.contactState, tmp, tmpv, NULL, NULL);

      fillDesireds(rs, root_d, foot_d);


      if(zyx[0]>0)
        temp_feet_b_w[1] = 0.03;


      if(_stateTime>0.05&&abs(temp_feet_b_w[0]) < 0.07&&abs(temp_feet_b_w[1]) < 0.05)
      {
        if(abs(feet_b_w[RIGHT][0])>0.05 || abs(feet_b_w[RIGHT][1])>0.05) 
        {
          printf("dCoP changed !!!!!!!!!!!!!!!!!!11\n");
          if(feet_b_w[RIGHT][0]*feet_b_w[RIGHT][1]>=0)
          {
            if(zyx[1]>=0)
            {
              temp_feet_b_w[0] += -0.0005;
              temp_feet_b_w[1] += -0.0005;
            }else
            {
              temp_feet_b_w[0] += 0.0005;
              temp_feet_b_w[1] += 0.0005;
            }
          }else
          {
            if(zyx[1]>=0)
            {
              temp_feet_b_w[0] += -0.0005;
              temp_feet_b_w[1] += 0.0005;
            }else
            {
              temp_feet_b_w[0] += 0.0005;
              temp_feet_b_w[1] += -0.0005;
            }
          }
          printf("x = %8.4f   y  = %8.4f \n", temp_feet_b_w[0], temp_feet_b_w[1]);
        }
      }

      idcmd.b_cop[RIGHT][XX] = temp_feet_b_w[0];
      idcmd.b_cop[RIGHT][YY] = temp_feet_b_w[1];
      
      angSame = _curFeet[LEFT].rot.isApprox(tmpQ, 5e-1);
      negQ = _curFeet[LEFT].rot;
      neg_quat(negQ);
      angSame |= negQ.isApprox(tmpQ, 5e-1); 

      if (((_curFeet[LEFT].pos-_nxtFootStep.pos).segment<2>(0).norm() < 5e-2 && 
           angSame &&
           // quatDist(_curFeet[LEFT].rot, tmpQ) < 0.3 && 
           fabs(_curFeet[LEFT].pos[ZZ]-_nxtFootStep.pos[ZZ]) < 4e-2) ||
           _stateTime > 7) 
      {
        gotoState(LowerLeftFoot);
      }
      break;

    case LiftRightFoot:
      t = _stateTime / ss_duration;
      t = std::min(t, 1.);
      t = std::max(t, 0.);

      t1 = (_stateTime - 0.2*ss_duration) / (0.8*ss_duration);
      t1 = std::min(t1, 1.);
      t1 = std::max(t1, 0.);

      zyx = quat2zyx(_curFeet[LEFT].rot);
      //tmpPos = _stanceFeet[RIGHT].pos*(1-t) + t*_nxtFootStep.pos;


      //tmpPos = _stanceFeet[RIGHT].pos + (1-cos(2*M_PI*t1))/2*(_nxtFootStep.pos-_stanceFeet[RIGHT].pos);
      //tmpPos[2] = _stanceFeet[RIGHT].pos[2] + (1-cos(2*M_PI*t))/2*(_nxtFootStep.pos[2] - _stanceFeet[RIGHT].pos[2]);

      tmpPos = _stanceFeet[RIGHT].pos + t1*(_nxtFootStep.pos-_stanceFeet[RIGHT].pos);
      tmpPos[2] = _stanceFeet[RIGHT].pos[2] + t*(_nxtFootStep.pos[2] - _stanceFeet[RIGHT].pos[2]);

      tmpVel = Eigen::Vector3d::Zero();

      if(t1 ==0) {
        tmpQ = _stanceFeet[RIGHT].rot; // _curFeet[RIGHT].rot;
        tmpVel[2] = 0.1;
      }
      else
        //tmpQ = zyx2quat(Eigen::Vector3d(0,zyx[1],_heading));
        tmpQ = _stanceFeet[RIGHT].rot.slerp(t, zyx2quat(Eigen::Vector3d(0,zyx[1],_heading)));

      foot_d = TrajPoint<7,0>(rs.time, rs.contactState, tmp, tmpv, NULL, NULL);

      t = (rs.time-lift_start_time) / (lift_end_time-lift_start_time);
      t = std::min(t, 1.);
      t = std::max(t, 0.);

      tmpPos = (1-t)*Eigen::Map<const Eigen::Vector3d>(root_d.pos) + t*(_curFeet[LEFT].pos+Eigen::Vector3d(0,0,com_z));
      //tmpPos = _curFeet[LEFT].pos;
      //tmpPos[ZZ] += com_z;
      tmpVel = Eigen::Vector3d::Zero();
      tmpQ = Eigen::Map<const Eigen::Quaterniond>(_dsComTraj[rootIdx].pos+3);
      root_d = TrajPoint<8,0>(rs.time, rs.contactState, tmp, tmpv, NULL, NULL);

      fillDesireds(rs, root_d, foot_d);


      if(zyx[0]<0)
        temp_feet_b_w[1] = -0.03;


      if(_stateTime>0.05&&abs(temp_feet_b_w[0]) < 0.07&&abs(temp_feet_b_w[1]) < 0.05)
      {
        if(abs(feet_b_w[LEFT][0])>0.05 || abs(feet_b_w[LEFT][1])>0.05) 
        {
          printf("dCoP changed !!!!!!!!!!!!!!!!!!\n");
          if(feet_b_w[LEFT][0]*feet_b_w[LEFT][1]>=0)
          {
            if(zyx[1]>=0)
            {
              temp_feet_b_w[0] += -0.0005;
              temp_feet_b_w[1] += -0.0005;
            }else
            {
              temp_feet_b_w[0] += 0.0005;
              temp_feet_b_w[1] += 0.0005;
            }
          }else
          {
            if(zyx[1]>=0)
            {
              temp_feet_b_w[0] += -0.0005;
              temp_feet_b_w[1] += 0.0005;
            }else
            {
              temp_feet_b_w[0] += 0.0005;
              temp_feet_b_w[1] += -0.0005;
            }
          }
          printf("x = %8.4f   y  = %8.4f \n", temp_feet_b_w[0], temp_feet_b_w[1]);
        }
      }

      idcmd.b_cop[LEFT][XX] = temp_feet_b_w[0];
      idcmd.b_cop[LEFT][YY] = temp_feet_b_w[1];

      angSame = _curFeet[RIGHT].rot.isApprox(tmpQ, 5e-1);
      negQ = _curFeet[RIGHT].rot;
      neg_quat(negQ);
      angSame |= negQ.isApprox(tmpQ, 5e-1);

      if (((_curFeet[RIGHT].pos-_nxtFootStep.pos).segment<2>(0).norm() < 5e-2 && 
           //quatDist(_curFeet[RIGHT].rot, tmpQ) < 0.3 && 
           angSame &&
           fabs(_curFeet[RIGHT].pos[ZZ]-_nxtFootStep.pos[ZZ]) < 4e-2) || 
           _stateTime > 7) 
      {
        gotoState(LowerRightFoot);
      }
      break;

    case LowerLeftFoot:
      tmpPos = _nxtFootStep.pos;
      tmpPos[ZZ] = _curFeet[LEFT].pos[ZZ] - ss_lower_z_rate;
      tmpVel = Eigen::Vector3d(0, 0, 0);
      zyx = quat2zyx(_curFeet[RIGHT].rot);
      tmpQ = zyx2quat(Eigen::Vector3d(0,zyx[1],_heading));

      foot_d = TrajPoint<7,0>(rs.time, rs.contactState, tmp, tmpv, NULL, NULL);

      tmpPos = _curFeet[RIGHT].pos;
      tmpPos[ZZ] += com_z;
      tmpVel = Eigen::Vector3d::Zero();
      tmpQ = Eigen::Map<const Eigen::Quaterniond>(_dsComTraj[rootIdx].pos+3);
      root_d = TrajPoint<8,0>(rs.time, rs.contactState, tmp, tmpv, NULL, NULL);

      fillDesireds(rs, root_d, foot_d);

      idcmd.b_cop[RIGHT][XX] = temp_feet_b_w[0];
      idcmd.b_cop[RIGHT][YY] = temp_feet_b_w[1];

      // check if foot not moving
      if ((_stateTime > 1) && 
          ((feet_vel[LEFT].norm() < 5e-3 && feet_w[LEFT].norm() < 5e-3) || rs.feet[LEFT].b_F[ZZ] > rs.m * GRAVITY * touch_down_fz_thres))
      { 
        printf("l v norm %g w norm %g, fz %g\n", feet_vel[LEFT].norm(), feet_w[LEFT].norm(), rs.feet[LEFT].b_F[ZZ]);
        _stepCount++;
        if (_stopping == LastSwing) {
          _stopping = NoStop;
          _idle = true;
          gotoState(Idle);
          setupDs(rs, RIGHT, -1); 
        }
        else {
          gotoState(ShiftComToLeft);
          setupDs(rs, RIGHT, LEFT); 
        }
        //gotoState(TouchDownLeft);
      }
      break;

    case LowerRightFoot:
      tmpPos = _nxtFootStep.pos;
      tmpPos[ZZ] = _curFeet[RIGHT].pos[ZZ] - ss_lower_z_rate;
      tmpVel = Eigen::Vector3d(0, 0, 0); 
      zyx = quat2zyx(_curFeet[LEFT].rot);
      tmpQ = zyx2quat(Eigen::Vector3d(0,zyx[1],_heading));

      foot_d = TrajPoint<7,0>(rs.time, rs.contactState, tmp, tmpv, NULL, NULL);

      tmpPos = _curFeet[LEFT].pos;
      tmpPos[ZZ] += com_z;
      tmpVel = Eigen::Vector3d::Zero();
      tmpQ = Eigen::Map<const Eigen::Quaterniond>(_dsComTraj[rootIdx].pos+3);
      root_d = TrajPoint<8,0>(rs.time, rs.contactState, tmp, tmpv, NULL, NULL);

      fillDesireds(rs, root_d, foot_d);

      idcmd.b_cop[LEFT][XX] = temp_feet_b_w[0];
      idcmd.b_cop[LEFT][YY] = temp_feet_b_w[1];

      // check if foot not moving
      if ((_stateTime > 1) && 
          ((feet_vel[RIGHT].norm() < 5e-3 && feet_w[RIGHT].norm() < 5e-3) || rs.feet[RIGHT].b_F[ZZ] > rs.m * GRAVITY * touch_down_fz_thres))
      {
        printf("r v norm %g w norm %g, fz %g\n", feet_vel[RIGHT].norm(), feet_w[RIGHT].norm(), rs.feet[RIGHT].b_F[ZZ]);
        _stepCount++;
        if (_stopping == LastSwing) {
          _stopping = NoStop;
          _idle = true;
          gotoState(Idle);
          setupDs(rs, LEFT, -1); 
        }
        else {
          gotoState(ShiftComToRight);
          setupDs(rs, LEFT, RIGHT);
        }
        //gotoState(TouchDownRight);
      }
      break;

      /*
         case TouchDownLeft:
      // weight distribution
      root_d.pos[7] = 0.2;
      fillDesireds(rs, root_d, foot_d);

      if ((rootd.norm() < 5e-3 && rootw.norm() < 5e-3) || (_stateTime > 0.5)) {
      if (_stopping == LastSwing) {
      _stopping = NoStop;
      _idle = true;
      setupDs(rs, LEFT, true); 
      gotoState(Idle);
      }
      else {
      setupDs(rs, LEFT); 
      gotoState(ShiftComToLeft);
      }
      }
      break;

      case TouchDownRight:
      root_d.pos[7] = 0.8;
      fillDesireds(rs, root_d, foot_d);

      if ((rootd.norm() < 5e-3 && rootw.norm() < 5e-3) || (_stateTime > 0.5)) {
      if (_stopping == LastSwing) {
      _stopping = NoStop;
      _idle = true;
      setupDs(rs, RIGHT, true); 
      gotoState(Idle);
      }
      else {
      setupDs(rs, RIGHT);
      gotoState(ShiftComToRight);
      }
      }
      break;
      */
  }

  if (!idCon->ID(rs, idcmd, cmd))
    return -1;

/*
  if (_state == TouchDownLeft || !isSwing(rs.contactState, LEFT) || 
      (_state == LiftLeftFoot && _stateTime < 0.2*ss_duration)) {
    cmd.controls_ff[A_L_LEG_LAX] -= stance_lax_d*(rs.jointsd[A_L_LEG_LAX]);
    cmd.controls_ff[A_L_LEG_UAY] -= stance_uay_d*(rs.jointsd[A_L_LEG_UAY]);
    cmd.controls_ff[A_L_LEG_UHZ] -= stance_uhz_d*(rs.jointsd[A_L_LEG_UHZ]);
  }
  if (_state == TouchDownRight || !isSwing(rs.contactState, RIGHT) ||
     (_state == LiftRightFoot && _stateTime < 0.2*ss_duration)) {
    cmd.controls_ff[A_R_LEG_LAX] -= stance_lax_d*(rs.jointsd[A_R_LEG_LAX]);
    cmd.controls_ff[A_R_LEG_UAY] -= stance_uay_d*(rs.jointsd[A_R_LEG_UAY]);
    cmd.controls_ff[A_R_LEG_UHZ] -= stance_uhz_d*(rs.jointsd[A_R_LEG_UHZ]);
  }
*/

  return 0;
}


void StaticWalkingCon::startWalking(const RobotState &rs, double heading, double dist)
{
  printf("\n\n\nSWC: starting, heading %g, dist %g\n\n\n", heading, dist);
  _stepCount = 0;
  _terrainMode = Flat;
  _heading = heading;
  _stopping = NoStop;
  _idle = false;
  
  _distTraveled = 0;
  _maxDist = dist;
  //_iniPos = Eigen::Vector2d((rs.feet[LEFT].w_sensor_pos[XX] + rs.feet[RIGHT].w_sensor_pos[XX])/2., 
    //                        (rs.feet[LEFT].w_sensor_pos[YY] + rs.feet[RIGHT].w_sensor_pos[YY])/2.);
}

void StaticWalkingCon::stop()
{
  if (_stopping == NoStop) {
    printf("\n\n\nSWC: Stop signal caught.\n\n\n");
    _stopping = StopSignaled;
  }
}

bool StaticWalkingCon::setupDs(const RobotState &rs, int start, int end)
{
  knot_t k0[3];
  knot_t k1[3];
  double pos0[9] = {0}, pos1[9] = {0};
  double t0 = rs.time;
  double t1 = t0 + ds_duration;
  double dt = rs.timeStep;
  double slerpt;

  for (int i = 0; i < LR; i++) {
    _stanceFeet[i].pos = Eigen::Map<const Eigen::Vector3d> (rs.feet[i].w_sensor_pos);
    _stanceFeet[i].rot = rs.feet[i].w_q;
  }

  // setup orientation knots
  Eigen::Quaterniond root_q0 = rs.rootq;
  Eigen::Quaterniond root_q1; 
  // setup pos knots
  dvec_copy(pos0, rs.com, 3);
  dvec_copy(pos0+3, rs.comd, 3);

  // set terrainMode
  double xy_dist = (_stanceFeet[LEFT].pos-_stanceFeet[RIGHT].pos).segment<2>(0).norm();  
  double dz;
  if (end == LEFT || end == -1) {
    dz = _stanceFeet[LEFT].pos[ZZ] - _stanceFeet[RIGHT].pos[ZZ];
    dvec_copy(pos1, _stanceFeet[LEFT].pos.data(), 3);
  }
  else {
    dz = _stanceFeet[RIGHT].pos[ZZ] - _stanceFeet[LEFT].pos[ZZ];
    dvec_copy(pos1, _stanceFeet[RIGHT].pos.data(), 3);
  }

  if (dz / xy_dist < -0.05)
    _terrainMode = DownHill;
  else if (dz / xy_dist > 0.05)
    _terrainMode = UpHill;
  else
    _terrainMode = Flat;

  printf("terrain mode %s\n", modeNames[_terrainMode].c_str());
  //printf("terrain mode %s\n", terrainModeNames[_terrainMode].c_str());

  if (_terrainMode == UpHill) {
    root_q1 = zyx2quat(Eigen::Vector3d(0, up_torso_pitch, _heading));
    pos1[ZZ] += up_com_z;
  }
  else {
    root_q1 = zyx2quat(Eigen::Vector3d(0, 0, _heading));
    pos1[ZZ] += com_z;
  }
  if (root_q1.dot(root_q0) < 0)
    neg_quat(root_q1);

  if (end == -1) {
    Eigen::Vector3d tmp = (_stanceFeet[LEFT].pos+_stanceFeet[RIGHT].pos) / 2.;
    dvec_copy(pos1, tmp.data(), 3);
    pos1[ZZ] += com_z;
  }

  set_knots_pos_only(t0, t1, pos0, pos1, pos0+3, pos1+3, pos0+6, pos1+6, k0, k1);

  // interp
  _dsComTraj.reset();
  double tmp[8], tmpv[8], tmpacc[8], dist[LR];
  Eigen::Map <Eigen::Vector3d>  tmpP(tmp);
  Eigen::Map <Eigen::Quaterniond> tmpQ(tmp+3);
  Eigen::Map <Eigen::Vector3d> tmpW(tmpv+3);
  Eigen::AngleAxisd root_wQ = Eigen::AngleAxisd(root_q1 * (root_q0.inverse()));
  Eigen::Vector3d root_w = root_wQ.axis()*(root_wQ.angle()/(t1-t0)); 

  printf("t0 %g, t1 %g, com0 (%g %g %g, %g %g %g %g), com1 (%g %g %g, %g %g %g %g)\n", 
      t0, t1, pos0[0], pos0[1], pos0[2], root_q0.w(), root_q0.x(), root_q0.y(), root_q0.z(), 
      pos1[0], pos1[1], pos1[2], root_q1.w(), root_q1.x(), root_q1.y(), root_q1.z());
  printf("fl (%g %g %g, %g %g %g %g), fr (%g %g %g, %g %g %g %g)\n", 
      _stanceFeet[LEFT].pos[0], _stanceFeet[LEFT].pos[1], _stanceFeet[LEFT].pos[2], _stanceFeet[LEFT].rot.w(), _stanceFeet[LEFT].rot.x(), _stanceFeet[LEFT].rot.y(), _stanceFeet[LEFT].rot.z(), 
      _stanceFeet[RIGHT].pos[0], _stanceFeet[RIGHT].pos[1], _stanceFeet[RIGHT].pos[2], _stanceFeet[RIGHT].rot.w(), _stanceFeet[RIGHT].rot.x(), _stanceFeet[RIGHT].rot.y(), _stanceFeet[RIGHT].rot.z());

  knot_t wl0, wl1;
  double wl[2];
  switch (end) {
    case LEFT:
      wl[1] = 1;
      break;
    case RIGHT:
      wl[1] = 0;
      break;
    case -1:
      wl[1] = 0.5;
      break;
  }

  switch (start) {
    case LEFT:
      wl[0] = 1;
      break;
    case RIGHT:
      wl[0] = 0;
      break;
    case -1:
      wl[0] = 0.5;
      break;
  }

  set_single_knot(t0, t1, wl[0], wl[1], &wl0, &wl1);

  for (double t = t0; t < t1 - dt/2.; t += dt) 
  {
    for (int i = 0; i < 8; i++)
      tmp[i] = tmpv[i] = tmpacc[i] = 0;

    // slerp rotation
    slerpt = (t-t0)/(t1-t0);
    slerpt = std::min(slerpt, 1.);
    slerpt = std::max(slerpt, 0.);
    tmpQ = root_q0.slerp(slerpt, root_q1).normalized();
    tmpW = root_w; 

    // quintic spline position
    lookup_pos_only(t, k0, k1, tmp, tmpv, tmpacc);
    
    // compute weight distribution
    dist[LEFT] = (tmpP-_stanceFeet[LEFT].pos).segment<2>(0).norm();
    dist[RIGHT] = (tmpP-_stanceFeet[RIGHT].pos).segment<2>(0).norm();
 
    tmp[7] = 1 - dist[LEFT] / (dist[LEFT]+dist[RIGHT]);

    tmp[7] = std::min(tmp[7], w_high_thres);
    tmp[7] = std::max(tmp[7], w_low_thres);
    /*
    quintic_spline(t, &wl0, &wl1, tmp+7, NULL, NULL);
    tmp[7] = std::min(tmp[7], w_high_thres);
    tmp[7] = std::max(tmp[7], w_low_thres);
    */

    _dsComTraj.append(t, DSc, tmp, tmpv, tmpacc, NULL);
  }

  return true;
}

void StaticWalkingCon::fillDesireds(const RobotState &rs, const TrajPoint<8,0> &root, const TrajPoint<7,0> &foot)
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
      // bdi upper body joints
      if (i == A_L_ARM_USY) ikcmd.joints[i] = 0.3;
      if (i == A_L_ARM_USY) ikcmd.joints[i] = 0.5-0.24;
      if (i == A_L_ARM_SHX) ikcmd.joints[i] = -1.3; 
      if (i == A_L_ARM_ELY) ikcmd.joints[i] = 2 ; 
      if (i == A_L_ARM_ELX) ikcmd.joints[i] = 0.5 + 1; 
      if (i == A_R_ARM_USY) ikcmd.joints[i] = 0.3;
      if (i == A_R_ARM_USY) ikcmd.joints[i] = 0.5-0.24;
      if (i == A_R_ARM_SHX) ikcmd.joints[i] = 1.3; 
      if (i == A_R_ARM_ELY) ikcmd.joints[i] = 2 ; 
      if (i == A_R_ARM_ELX) ikcmd.joints[i] = -0.5 - 1;  

      if (i == A_NECK_AY)
        ikcmd.joints[i] = _neck_ay_d;

      if (i == A_BACK_MBY) {
        ikcmd.joints[i] = 0;
      }

      ikcmd.jointsd[i] = 0;
      idcmd.jointsdd[i] = 0 
        + fd_joint_p*(ikcmd.joints[i]-rs.joints[i]) 
        + fd_joint_d*(ikcmd.jointsd[i]-rs.jointsd[i]); 
    }
  }

  // relative zmp
  idcmd.b_cop[LEFT][XX] = 0;
  idcmd.b_cop[LEFT][YY] = 0;
  idcmd.b_cop[RIGHT][XX] = 0;
  idcmd.b_cop[RIGHT][YY] = 0;

  // weight distribution
  switch (rs.contactState) {
    case DSl:
    case DSr:
    case DSc:
      idcmd.wl = root.pos[7];
      break;
    case SSL:
      idcmd.wl = 1;
      break;
    case SSR:
      idcmd.wl = 0;
      break;
    default:
      assert(0);
  }

  //////////////////////////////////////////////////////////////////////////////////////////
  // root 
  for (int i = 0; i < 3; i++) {
    ikcmd.com[i] = root.pos[i];
    ikcmd.comd[i] = root.vel[i];
    idcmd.comdd[i] = root.acc[i] 
      + fd_com_p[i]*(ikcmd.com[i]-rs.com[i]) 
      + fd_com_d[i]*(ikcmd.comd[i]-rs.comd[i]);

    ikcmd.root[i] = INFINITY; 
    ikcmd.rootd[i] = INFINITY;
    idcmd.rootdd[i] = INFINITY;
  }

  // orientation tracking
  ikcmd.torsoQ = Eigen::Map<const Eigen::Quaterniond> (root.pos+3);
  if (ikcmd.torsoQ.dot(rs.utorsoq) < 0)
    neg_quat(ikcmd.torsoQ);
  Eigen::AngleAxisd rotVec(rs.utorsoq * (ikcmd.torsoQ.inverse().normalized()));
  Eigen::Vector3d Rotvec = rotVec.axis() * rotVec.angle();

  for (int i = 0; i < 3; i++) {
    ikcmd.torsoW[i] = root.vel[i+3];
    idcmd.torsoWd[i] = 0 
      - fd_com_p[i+3]*(Rotvec[i]) 
      + fd_com_d[i+3]*(ikcmd.torsoW[i]-rs.utorsow[i]);
  }

  // desired orientation in body frame
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
  Eigen::Quaterniond world_to_body = rs.rootq.inverse().normalized();
  Eigen::Quaterniond body_q_d = world_to_body * ikcmd.pelvQ;
  if (body_q_d.dot(Eigen::Quaterniond::Identity()) < 0) 
    neg_quat(body_q_d);
  rotVec = Eigen::Quaterniond::Identity() * (body_q_d.inverse().normalized());
  Rotvec = rotVec.axis() * rotVec.angle();

  Eigen::Vector3d body_w_d = world_to_body.toRotationMatrix() * Eigen::Map<const Eigen::Vector3d> (root.vel+3);

  for (int i = 0; i < 3; i++) {
    ikcmd.root_b_w[i] = body_w_d[i];
    idcmd.root_b_wd[i] = 0
      - fd_com_p[i]*(Rotvec[i])
      + fd_com_d[i]*(ikcmd.root_b_w[i]-rs.root_b_w[i]);
  }
  // world frame angular velocity and acceleration
  dvec_copy(ikcmd.rootd+3, root.vel+3, 3);
  Eigen::Map <Eigen::Vector3d> root_w_wd_d(idcmd.pelvWd+3);
  Eigen::Map <Eigen::Vector3d> root_b_wd_d(idcmd.root_b_wd);
  root_w_wd_d = rs.rootq.toRotationMatrix() * root_b_wd_d;
  */

  /////////////////////////////////////////////////////////////
  for (int side = 0; side < LR; side++) 
  {
    dvec_copy(ikcmd.foot[side], foot.pos, 3);
    ikcmd.footQ[side] = Eigen::Map<const Eigen::Quaterniond> (foot.pos+3);
    // stance
    for (int i = 0; i < 6; i++) {
      ikcmd.footd[side][i] = 0;
      idcmd.footdd[side][i] = 0;
    }

    // left foot position
    // swing
    if (isSwing(rs.contactState, side)) {
      for (int j = 0; j < 3; j++) {
        ikcmd.footd[side][j] = foot.vel[j];
        idcmd.footdd[side][j] = foot.acc[j]
          + fd_foot_p[j]*(ikcmd.foot[side][j]-rs.feet[side].w_sensor_pos[j]);

        idcmd.footdd[side][j] += fd_foot_d[j]*(ikcmd.footd[side][j]-rs.feet[side].w_sensor_vel[j]);

        if (_mudMode) {
          _swingFeetPosIntegral[side][j] += (ikcmd.foot[side][j]-rs.feet[side].w_sensor_pos[j]);
          idcmd.footdd[side][j] += fd_foot_i[j]*_swingFeetPosIntegral[side][j];
        }
      }
    }  

    // foot orientation
    // swing
    if (isSwing(rs.contactState, side)) 
    {
      if (ikcmd.footQ[side].dot(rs.feet[side].w_q) < 0) 
        neg_quat(ikcmd.footQ[side]);
      rotVec = rs.feet[side].w_q * (ikcmd.footQ[side].inverse().normalized());
      Rotvec = rotVec.axis() * rotVec.angle();

      for (int i = 3; i < 6; i++) {
        ikcmd.footd[side][i] = foot.vel[i+3];
        idcmd.footdd[side][i] = 0;
        idcmd.footdd[side][i] += fd_foot_d[i]*(ikcmd.footd[side][i]-rs.feet[side].w_sensor_vel[i]);

        //if (rs.feet[side].b_F[ZZ] < 20)
        idcmd.footdd[side][i] -= fd_foot_p[i]*Rotvec[i-3];
        //if (rs.feet[side].b_F[ZZ] >= 20 && ((side == LEFT && _state == LowerLeftFoot) || (side == RIGHT && _state == LowerRightFoot)))
         // idcmd.footdd[side][i] += fd_foot_p[i]*Rotvec[i-3];

        if (_mudMode) {
          _swingFeetRotIntegral[side][i-3] += Rotvec[i-3];
          idcmd.footdd[side][i] -= fd_foot_i[i]*_swingFeetRotIntegral[side][i-3];
        }
      }
    }
  }
}

void StaticWalkingCon::buildLookup()
{
  _conName = "SWC";
  std::string name;

  _lookup["fd_joint_p"] = &fd_joint_p;
  _lookup["fd_joint_d"] = &fd_joint_d;

  _lookup["step_length"] = &step_length;
  _lookup["step_width"] = &step_width;
  _lookup["com_z"] = &com_z;
  _lookup["ds_duration"] = &ds_duration;
  _lookup["ss_duration"] = &ss_duration;
  _lookup["ss_height"] = &ss_height;
  _lookup["ss_lower_z_rate"] = &ss_lower_z_rate;

  _lookup["up_foot_pitch"] = &up_foot_pitch;
  _lookup["up_torso_pitch"] = &up_torso_pitch;
  _lookup["up_ss_height"] = &up_ss_height;
  _lookup["up_step_length"] = &up_step_length;
  _lookup["up_com_z"] = &up_com_z;

  _lookup["stance_lax_d"] = &stance_lax_d;
  _lookup["stance_uay_d"] = &stance_uay_d;
  _lookup["stance_uhz_d"] = &stance_uhz_d;
  _lookup["touch_down_fz_thres"] = &touch_down_fz_thres;
  _lookup["lift_off_fz_thres"] = &lift_off_fz_thres;
  _lookup["w_high_thres"] = &w_high_thres;
  _lookup["w_low_thres"] = &w_low_thres;

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
    name = std::string("fd_foot_i_");
    name.append(boost::to_string(i));
    _lookup[name] = fd_foot_i+i;
  }
}


/*
   bool StaticWalkingCon::setupLift(const RobotState &rs, int side)
   {
   knot_t k0[3];
   knot_t k1[3];
// 3 pos, 3 vel, 3 acc
double pos0[9] = {0}, pos1[9] = {0};
double t0 = rs.time;
double t1 = t0 + ss_duration / 2.;
double dt = rs.timeStep;
double slerpt; 

assert(rs.feet[LEFT].getRefPoint() == RefCenter && 
rs.feet[RIGHT].getRefPoint() == RefCenter);

// setup orientation knots
Eigen::Quaterniond foot_q0 = rs.feet[side].w_q;
Eigen::Quaterniond foot_q1 = zyx2quat(Eigen::Vector3d(0, 0, _heading));

// setup pos knots
dvec_copy(pos0, _stanceFeet[side].pos.data(), 3);
//dvec_copy(pos0, rs.feet[side].w_sensor_pos, 3);
//dvec_copy(pos0+3, rs.feet[side].w_sensor_vel, 3);
dvec_copy(pos1, pos0, 3);

if (_stepCount == 0) {
pos1[XX] += step_length*cos(_heading);
pos1[YY] += step_length*sin(_heading);
}
else {
pos1[XX] += 2*step_length*cos(_heading);
pos1[YY] += 2*step_length*sin(_heading);
}

pos1[ZZ] += ss_height;
//pos0[6+ZZ] = 10;
set_knots_pos_only(t0, t1, pos0, pos1, pos0+3, pos1+3, pos0+6, pos1+6, k0, k1);

// interp
_ssFootTraj.reset();
double tmp[7], tmpv[7], tmpacc[7];
Eigen::Map <Eigen::Vector3d>  tmpP(tmp);
Eigen::Map <Eigen::Quaterniond> tmpQ(tmp+3);
Eigen::Map <Eigen::Vector3d> tmpW(tmpv+3);
Eigen::AngleAxisd foot_wQ = Eigen::AngleAxisd(foot_q1 * (foot_q0.inverse()));
Eigen::Vector3d foot_w = foot_wQ.axis()*(foot_wQ.angle()/(t1-t0)); 

printf("t0 %g, t1 %g, foot0 (%g %g %g), foot1 (%g %g %g)\n", t0, t1, pos0[0], pos0[1], pos0[2], pos1[0], pos1[1], pos1[2]);

for (double t = t0; t < t1 - dt/2.; t += dt) 
{
for (int i = 0; i < 7; i++)
tmp[i] = tmpv[i] = tmpacc[i] = 0;

// slerp rotation
slerpt = (t-t0)/(t1-t0);
slerpt = std::min(slerpt, 1.);
slerpt = std::max(slerpt, 0.);
tmpQ = foot_q0.slerp(slerpt, foot_q1).normalized();
tmpW = foot_w; 

// quintic spline position
lookup_pos_only(t, k0, k1, tmp, tmpv, tmpacc);

_ssFootTraj.append(t, _contactState, tmp, tmpv, tmpacc, NULL);
}

return true; 
}

bool StaticWalkingCon::setupLower(const RobotState &rs, int side)
{
  knot_t k0[3];
  knot_t k1[3];
  // 3 pos, 3 vel, 3 acc
  double pos0[9] = {0}, pos1[9] = {0};
  double t0 = rs.time;
  double t1 = t0 + ss_duration / 2.;
  double dt = rs.timeStep;
  double slerpt; 

  assert(rs.feet[LEFT].getRefPoint() == RefCenter && 
      rs.feet[RIGHT].getRefPoint() == RefCenter);

  // setup orientation knots
  Eigen::Quaterniond foot_q0 = rs.feet[side].w_q;
  Eigen::Quaterniond foot_q1 = zyx2quat(Eigen::Vector3d(0, 0, _heading));

  // setup pos knots
  //dvec_copy(pos0, _stanceFeet[side].pos.data(), 3);
  dvec_copy(pos0, rs.feet[side].w_sensor_pos, 3);
  dvec_copy(pos0+3, rs.feet[side].w_sensor_vel, 3);

  dvec_copy(pos1, _stanceFeet[side].pos.data(), 3);

  if (_stepCount == 0) {
    pos1[XX] += step_length*cos(_heading);
    pos1[YY] += step_length*sin(_heading);
  }
  else {
    pos1[XX] += 2*step_length*cos(_heading);
    pos1[YY] += 2*step_length*sin(_heading);
  }

  // pos0[ZZ] += ss_height;
  // pos1[ZZ] -= 0.05; // ss_height;

  set_knots_pos_only(t0, t1, pos0, pos1, pos0+3, pos1+3, pos0+6, pos1+6, k0, k1);

  // interp
  _ssFootTraj.reset();
  double tmp[7], tmpv[7], tmpacc[7];
  Eigen::Map <Eigen::Vector3d>  tmpP(tmp);
  Eigen::Map <Eigen::Quaterniond> tmpQ(tmp+3);
  Eigen::Map <Eigen::Vector3d> tmpW(tmpv+3);
  Eigen::AngleAxisd foot_wQ = Eigen::AngleAxisd(foot_q1 * (foot_q0.inverse()));
  Eigen::Vector3d foot_w = foot_wQ.axis()*(foot_wQ.angle()/(t1-t0)); 

  printf("t0 %g, t1 %g, foot0 (%g %g %g), foot1 (%g %g %g)\n", t0, t1, pos0[0], pos0[1], pos0[2], pos1[0], pos1[1], pos1[2]);

  for (double t = t0; t < t1 - dt/2.; t += dt) 
  {
    for (int i = 0; i < 7; i++)
      tmp[i] = tmpv[i] = tmpacc[i] = 0;

    // slerp rotation
    slerpt = (t-t0)/(t1-t0);
    slerpt = std::min(slerpt, 1.);
    slerpt = std::max(slerpt, 0.);
    tmpQ = foot_q0.slerp(slerpt, foot_q1).normalized();
    tmpW = foot_w; 

    // quintic spline position
    lookup_pos_only(t, k0, k1, tmp, tmpv, tmpacc);

    _ssFootTraj.append(t, _contactState, tmp, tmpv, tmpacc, NULL);
  }

  return true; 
}
*/ 

