/*
 *  Copyright 2013 Sasanka Nagavalli
 *  Based on code by Siyuan Feng
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include <fstream>
#include <iostream>

#include <eigen3/Eigen/Cholesky>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>

#include "traj.h"

#include "lqr_controller.hpp"
#include "lipm_planner.hpp"

LipmVarHeightPlanner::LipmVarHeightPlanner()
: _mass(0)
, _dt(0)
, _alpha(0)
, _Q(Eigen::Matrix<double,6,6>::Identity())
, _R(Eigen::Matrix<double,3,3>::Identity())
, _Vxx(Eigen::Matrix<double,6,6>::Identity())
, _lqr(LqrController<6,3>())
{}

LipmVarHeightPlanner::LipmVarHeightPlanner(double m, double dt, const Eigen::Matrix<double,6,6> &Q, const Eigen::Matrix<double,3,3> &R)
: _mass(m)
, _dt(dt)
, _alpha(1)
, _Q(Q)
, _R(R)
, _Vxx(Eigen::Matrix<double,6,6>::Identity())
, _lqr(LqrController<6,3>())
{}

LipmVarHeightPlanner &LipmVarHeightPlanner::operator= (const LipmVarHeightPlanner &other)
{
  if (&other == this)
    return *this;

  _mass = other._mass;
  _dt = other._dt;
  _alpha = other._alpha;

  _Q = other._Q;
  _R = other._R;
  
  _du = other._du;
  _K = other._K;
  _Vxx = other._Vxx;
  _lqr = other._lqr;

  _zmp_d = other._zmp_d;
  _traj0 = other._traj0;
  _z0 = other._z0;

  return *this;
}

LipmVarHeightPlanner::~LipmVarHeightPlanner()
{
  _K.clear();
  _du.clear();
}

// traj z part should be offset to ground, not abs z coord!!
void LipmVarHeightPlanner::setZMPTraj(const Traj<3,3> &traj, const std::vector <double> &z0)
{
  assert(traj.size() > 0 && traj.size() == z0.size());
  _zmp_d = traj;
  _z0 = z0;
 
  // get last step Vxx and initial Ks
  const TrajPoint<3,3> &end = traj[traj.size()-1];
  double x[6] = {0};
  double u[3] = {0};
  dvec_copy(x, end.x, 6);
  dvec_copy(u, end.u, 3);
 
  Eigen::Matrix<double,6,6> A;
  Eigen::Matrix<double,6,3> B;
  
  getAB(x, u, _z0[_z0.size()-1], A, B);
  
  _lqr.setQ(_Q);
  _lqr.setR(_R);
  _lqr.infTimeLQR(A, B);
  
  _Vxx = _lqr.getV();
  
  if (traj.size() == _du.size()) {
    for (size_t i = 0; i < traj.size(); i++) {
      _du[i].setZero();
      _K[i] = _lqr.getK();
    }
  }
  else {
    _du.clear();
    _K.clear();

    // allocate K and du 
    for (size_t i = 0; i < traj.size(); i++) {
      _du.push_back(Eigen::Matrix<double,3,1>::Zero());
      _K.push_back(_lqr.getK());
    }
  }

  // initial trajectory
  _traj0 = Traj<3,3>();
  for (size_t i = 0; i < _zmp_d.size(); i++) {
    const TrajPoint<3,3> &end = traj[i];
    dvec_copy(x, end.x, 6);
    dvec_copy(u, end.u, 3);
    _traj0.append(end.time, end.type, x, x+3, NULL, u);
  }

  Traj<3,3> tmpTraj;
  forwardPass(traj[0].x, tmpTraj);
  _traj0 = tmpTraj;

  assert(_zmp_d.size() == _du.size() && 
         _zmp_d.size() == _K.size() && 
         _zmp_d.size() == _traj0.size());  
}

Eigen::Vector3d LipmVarHeightPlanner::getCOMAcc(size_t t, const Eigen::Vector3d &com, const Eigen::Vector3d &comd) const
{
  if (t >= _traj0.size())
    return Eigen::Vector3d::Zero();
  
  Eigen::Matrix<double,6,1> x0;
  Eigen::Matrix<double,6,1> z;
  Eigen::Matrix<double,3,1> u;
 
  x0 << com, comd;

  for (int i = 0; i < 6; i++)
    z(i) = x0(i) - _traj0[t].x[i]; 

  // u = K*z + u_ref
  u = Eigen::Map<const Eigen::Vector3d>(_traj0[t].u) + _K[t]*z;
  return computeAcc(x0.data(), u.data(), _z0[t]);
}

void LipmVarHeightPlanner::getCOMState(size_t t, Eigen::Vector3d &com, Eigen::Vector3d &comd) const
{
  if (t >= _traj0.size()) {
    com = Eigen::Vector3d::Zero();
    comd = Eigen::Vector3d::Zero();
    return;
  }

  com = Eigen::Map<const Eigen::Vector3d> (_traj0[t].pos);
  comd = Eigen::Map<const Eigen::Vector3d> (_traj0[t].vel);
}

double LipmVarHeightPlanner::forwardPass(const double *x0, Traj<3,3> &traj1) const
{
  Eigen::Matrix<double,6,1> z;
  Eigen::Matrix<double,6,1> z1;
  Eigen::Matrix<double,3,1> u;

  traj1 = Traj<3,3>();
  for (size_t i = 0; i < _zmp_d.size(); i++)
    traj1.append(_zmp_d[i].time, _zmp_d[i].type, NULL, NULL, NULL, NULL);
  
  dvec_copy(traj1[0].x, x0, 6);
  
  double cost = 0;

  Eigen::Matrix<double,6,1> x_h;
  Eigen::Matrix<double,3,1> u_h;

  for (size_t t = 0; t < _zmp_d.size(); t++) {
    // x - xref
    for (int i = 0; i < 6; i++) {
      z(i) = traj1[t].x[i] - _traj0[t].x[i];
      x_h(i) = traj1[t].x[i] - _zmp_d[t].x[i];
    }

    // u = alpha*du + uref
    u = _alpha*_du[t];
    for (int i = 0; i < 3; i++)
      u(i) += _traj0[t].u[i];

    // u += K*z
    u += _K[t]*z;
    for (int i = 0; i < 3; i++)
      traj1[t].u[i] = u(i);

    for (int i = 0; i < 3; i++)
      u_h(i) = traj1[t].u[i] - _zmp_d[t].u[i];

    // compute cost
    cost += 0.5 * x_h.transpose() * _Q * x_h;
    cost += 0.5 * u_h.transpose() * _R * u_h;
     
    // integrate
    if (t < _zmp_d.size()-1)
      integrate(traj1[t].x, traj1[t].u, _z0[t], traj1[t].acc, traj1[1+t].x);
  }
  
  /*
  FILE *out = fopen("tmp/lipmz_traj0", "w"); 
  for (size_t t = 0; t < _traj0.size(); t++) {
    fprintf(out, "%g %g %g %g %g %g\n", _traj0[t].x[0], _traj0[t].x[1], _traj0[t].x[2], _traj0[t].u[0], _traj0[t].u[1], _traj0[t].u[2]);
  }
  fclose(out);
 
  out = fopen("tmp/lipmz_traj1", "w"); 
  for (size_t t = 0; t < traj1.size(); t++) {
    fprintf(out, "%g %g %g %g %g %g\n", traj1[t].x[0], traj1[t].x[1], traj1[t].x[2], traj1[t].u[0], traj1[t].u[1], traj1[t].u[2]);
  }
  fclose(out);
  */ 

  return cost;
}

void LipmVarHeightPlanner::backwardPass(const Traj<3,3> &com)
{
  Eigen::Matrix<double,6,6> Qxx;
  Eigen::Matrix<double,3,3> Quu;
  Eigen::Matrix<double,3,3> invQuu;
  Eigen::Matrix<double,3,6> Qux;
  Eigen::Matrix<double,6,3> Qxu;
  Eigen::Matrix<double,6,1> Qx;
  Eigen::Matrix<double,3,1> Qu;

  Eigen::Matrix<double,6,6> A;
  Eigen::Matrix<double,6,3> B;
  Eigen::Matrix<double,6,6> Vxx = _Vxx;
  Eigen::Matrix<double,6,1> Vx = Eigen::Matrix<double,6,1>::Zero();

  Eigen::Matrix<double,6,1> cur_x;
  Eigen::Matrix<double,3,1> cur_u;
  Eigen::Matrix<double,6,1> x_h;
  Eigen::Matrix<double,3,1> u_h;

  //NEW_GSL_MATRIX(tmpXX0, tmpXX0_d, tmpXX0_v, 6, 6);
  //NEW_GSL_MATRIX(tmpUX0, tmpUX0_d, tmpUX0_v, 3, 6);
 
  const double *x0, *u0;

  for (int t = (int)_K.size()-1; t >= 0; t--) {
    //printf("======================================\n");
    // get x u
    x0 = _traj0[t].x;
    u0 = _traj0[t].u;
    
    //printf("%g %g %g %g %g %g\n", x0[0], x0[1], x0[2], x0[3], x0[4], x0[5]);
    //printf("%g %g %g\n", u0[0], u0[1], u0[2]);
    
    for (int i = 0; i < 6; i++) {
      cur_x(i) = x0[i];
      x_h(i) = x0[i] - _zmp_d[t].x[i];
    }
    for (int i = 0; i < 3; i++) {
      cur_u(i) = u0[i];
      u_h(i) = u0[i] - _zmp_d[t].u[i];
    }
    
    // get A B
    getAB(x0, u0, _z0[t], A, B);
    
    //printGSLMatrix("A", A);
    //printGSLMatrix("B", B);
   
    // Qx = Q*x_hat + A'*Vx
    Qx = _Q*x_h + A.transpose()*Vx;

    // Qu = R*u_hat + B'*Vx
    Qu = _R*u_h + B.transpose()*Vx;

    // Qxx = Q + A'*Vxx*A
    Qxx = _Q + A.transpose()*Vxx*A;

    // Qxu = A'*Vxx*B
    Qxu = A.transpose()*Vxx*B;

    // Quu = R + B'*Vxx*B
    Quu = _R + B.transpose()*Vxx*B;

    // Qux = Qxu'
    Qux = B.transpose()*Vxx*A;

    // K = -inv(Quu)*Qux
    _K[t] = -(Quu.llt().solve(Qux));
    
    // du = -inv(Quu)*Qu
    _du[t] = -(Quu.llt().solve(Qu));

    // Vx = Qx + K'*Qu
    Vx = Qx + _K[t].transpose()*Qu;
    
    // Vxx = Qxx + Qxu*K
    Vxx = Qxx + Qxu*_K[t];
    
    //getchar();
    //assert(!hasInfNan(_K[t]));
    //assert(!hasInfNan(_du[t]));
  }
}

bool LipmVarHeightPlanner::getCOMTraj(const Traj<3,3> &zmp, const double *state,  const std::vector<double> &z0, Traj<3,3> &com)
{
  //FILE *out;

  setZMPTraj(zmp, z0);
  double cost, cost0;
  
  //zmp.toFile("tmp/lipmz_d", true, true);
  //_traj0.toFile("tmp/lipmz_ini", true, true);
  cost0 = INFINITY;
  
  //char buf[1000];
  //sprintf(buf, "/home/sfeng/papers/humanoids13/data/lipm_it0");
  //_traj0.toFile(buf, true, true);

  for (int i = 0; i < 50; i++) {
    //printf("it %d\n", i);
    backwardPass(_traj0);
    cost = forwardPass(state, com);
    _traj0 = com;
    
    //sprintf(buf, "/home/sfeng/papers/humanoids13/data/lipm_it%d", i);
    //_traj0.toFile(buf, true, true);
    
    //printf("cost %g\n", cost);
    assert(!isnan(cost) && !isinf(cost));

    if (fabs(cost - cost0) < 1e-3)
      break;

    cost0 = cost;
    //com.toFile("tmp/lipmz_it", true, true);

    //getchar();
  }

  return true;
}

Eigen::Vector3d LipmVarHeightPlanner::computeAcc(const double x0[6], const double u[3], double z0) const
{
  double x = x0[XX];
  double y = x0[YY];
  double z = x0[ZZ] - z0;

  double px = u[XX];
  double py = u[YY];
  double F = u[ZZ];
  
  return Eigen::Vector3d((x-px)*F/(_mass*z), (y-py)*F/(_mass*z), (F/_mass)-GRAVITY);
}

void LipmVarHeightPlanner::integrate(const double x0[6], const double u[3], double z0, double acc[3], double x1[6]) const
{
  Eigen::Map<Eigen::Vector3d> dd(acc);
  dd = computeAcc(x0, u, z0);

  for (int i = 0; i < 3; i++)
    x1[i] = x0[i] + x0[3+i]*_dt;
  for (int i = 0; i < 3; i++)
    x1[3+i] = x0[3+i] + dd[i]*_dt;
}
 
void LipmVarHeightPlanner::getAB(const double x0[6], const double u[3], double z0, Eigen::Matrix<double,6,6> &A, Eigen::Matrix<double,6,3> &B) const
{
  A.setIdentity();
  B.setZero();
 
  double x = x0[XX];
  double y = x0[YY];
  double z = x0[ZZ] - z0;

  double px = u[XX];
  double py = u[YY];
  double F = u[ZZ];

  // A
  A(0, 3) = _dt;
  A(1, 4) = _dt;
  A(2, 5) = _dt;

  A(3, 0) = F*_dt/(_mass*z);
  A(3, 2) = F*_dt*(px-x)/(_mass*z*z);

  A(4, 1) = F*_dt/(_mass*z);
  A(4, 2) = F*_dt*(py-y)/(_mass*z*z);
 
  // B
  B(3, 0) = -F*_dt/(_mass*z); 
  B(3, 2) = -(px-x)*_dt/(_mass*z); 
  
  B(4, 1) = -F*_dt/(_mass*z); 
  B(4, 2) = -(py-y)*_dt/(_mass*z);

  B(5, 2) = _dt/_mass; 
}

 
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
 
LipmConstHeightPlanner::LipmConstHeightPlanner(double z, double dt)
: _height(z)
, _dt(dt)
, _lqr(LqrController<2,1>())
{
  _Q.setIdentity();
  _R.setIdentity();
  _Q *= 1e-6;

  _A(0,0) = 1;
  _A(0,1) = _dt;
  _A(1,0) = _dt*GRAVITY/_height;
  _A(1,1) = 1;

  _B(0,0) = 0;
  _B(1,0) = -_dt*GRAVITY/_height;

  _lqr.setQ(_Q);
  _lqr.setR(_R);
  _lqr.infTimeLQR(_A, _B);

  _K = _lqr.getK();
  _Quu_inv = (_B.transpose()*_lqr.getV()*_B + _R).inverse();
  
#if 0
  std::cout << "A" << std::endl << _A << std::endl
            << "B" << std::endl << _B << std::endl
            << "K" << std::endl << _K << std::endl
            << "Quu_inv" << std::endl << _Quu_inv << std::endl;
#endif
}

LipmConstHeightPlanner::~LipmConstHeightPlanner()
{
  _du.clear();
  _Vx.clear(); 
}

double LipmConstHeightPlanner::forwardPass(const double *x0, Traj<1,1> &com) const
{
  Eigen::Matrix<double,2,1> z;
  Eigen::Matrix<double,1,1> u;
  
  com = Traj<1,1>();
  for (size_t i=0; i<_zmp_d.size(); i++) {
    com.append(_zmp_d[i].time, _zmp_d[i].type, NULL, NULL, NULL, NULL);
  }

  com[0].x[0] = x0[0];
  com[0].x[1] = x0[1];
  
#if 0
  std::cout << _com.size() << " " << _zmp_d.size() << std::endl;
#endif

  for (size_t i=0; i<_zmp_d.size()-1; i++) {
    z(0) = com[i].x[0] - _zmp_d[i].x[0];
    z(1) = com[i].x[1];
    u = _K*z + _du[i];
    com[i].u[0] = u(0);

    z = _A*z + _B*u; 
    com[i+1].x[0] = z(0) + _zmp_d[i].x[0];
    com[i+1].x[1] = z(1);
  }
  
#if 0
  std::fstream p_d("tmp/pd", std::fstream::out);
  std::fstream p_a("tmp/pa", std::fstream::out);
  std::fstream com_out("tmp/com", std::fstream::out);
  for (size_t i=0; i<_zmp_d.size(); i++) {
    p_d << _zmp_d[i].x[0] << std::endl;
    p_a << com[i].u[0] + _zmp_d[i].x[0] << std::endl;
    com_out << com[i].x[0] << std::endl;
  }  
  p_d.close();
  p_a.close();
  com_out.close();
#endif
  return 0;
}

void LipmConstHeightPlanner::backwardPass(const Traj<1,1> &com)
{
  Eigen::Matrix<double,2,1> z;
  Eigen::Matrix<double,1,1> u;

  Eigen::Matrix<double,2,1> Lx;
  Eigen::Matrix<double,1,1> Lu;
  Eigen::Matrix<double,2,1> Qx;
  Eigen::Matrix<double,1,1> Qu;

  for (int i=(int)_zmp_d.size()-1; i>0; i--) {
    z(0) = com[i].x[0] - _zmp_d[i].x[0];
    z(1) = com[i].x[1];
    u(0) = com[i].u[0];

    Lx = _Q*z;
    Lu = _R*u;

    Qx = _A.transpose()*_Vx[i] + Lx;    
    Qu = _B.transpose()*_Vx[i] + Lu;

    _Vx[i-1] = _K.transpose()*Qu + Qx;
    _du[i] = -_Quu_inv.transpose()*Qu;
  }

#if 0
  std::fstream du("tmp/du", std::fstream::out);
  for (size_t i=0; i<_du.size(); i++) {
    du << _du[i](0) << std::endl;
  }
  du.close();
#endif
}

void LipmConstHeightPlanner::setZMPTraj(const Traj<1,1> &traj) 
{ 
  _zmp_d = traj; 

  if (_du.size() == traj.size()) {
    for (size_t i=0; i<traj.size(); i++) {
      _Vx[i].setZero();
      _du[i].setZero();
    }
  }
  else {
    _du.clear();
    _Vx.clear();
    for (size_t i=0; i<traj.size(); i++) {
      _Vx.push_back(Eigen::Matrix<double,2,1>::Zero());
      _du.push_back(Eigen::Matrix<double,1,1>::Zero());
    }
  }
}

bool LipmConstHeightPlanner::getCOMTraj(const Traj<1,1> &zmpTraj, const double *x0, Traj<1,1> &traj)
{
  setZMPTraj(zmpTraj);

  Traj<1,1> firstTraj;
  forwardPass(x0, firstTraj);
  backwardPass(firstTraj);
  forwardPass(x0, traj);

  return true;
}
