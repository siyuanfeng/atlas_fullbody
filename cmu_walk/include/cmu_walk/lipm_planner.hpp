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

#ifndef LIPM_PLANNER_H
#define LIPM_PLANNER_H

#include <vector>

#include <eigen3/Eigen/Core>

#include "drc_common_defines.h"
#include "traj.h"
#include "lqr_controller.hpp"

class LipmConstHeightPlanner {
  private:
    double _height;
    double _dt;

    Eigen::Matrix<double,2,2> _A;
    Eigen::Matrix<double,2,1> _B;
    Eigen::Matrix<double,1,2> _K;

    Eigen::Matrix<double,2,2> _Q;
    Eigen::Matrix<double,1,1> _R;
    Eigen::Matrix<double,1,1> _Quu_inv;
    
    std::vector< Eigen::Matrix<double,1,1> > _du;
    std::vector< Eigen::Matrix<double,2,1> > _Vx;

    LqrController<2,1> _lqr;
    
    Traj<1,1> _zmp_d;    

    void setZMPTraj(const Traj<1,1> &traj);
    double forwardPass(const double *x0, Traj<1,1> &com) const;
    void backwardPass(const Traj<1,1> &com);
  
  public:
    LipmConstHeightPlanner(double z, double dt);
    ~LipmConstHeightPlanner();
    
    bool getCOMTraj(const Traj<1,1> &zmp, const double *state, Traj<1,1> &com);
}; 

class LipmVarHeightPlanner {
  private:
    double _mass;
    double _dt;
    double _alpha;
    
    Eigen::Matrix<double,6,6> _Q;
    Eigen::Matrix<double,3,3> _R;
    
    std::vector< Eigen::Matrix<double,3,1> > _du;
    std::vector <Eigen::Matrix<double,3,6> > _K;
    Eigen::Matrix<double,6,6> _Vxx;

    LqrController<6,3> _lqr;
    Traj<3,3> _zmp_d;    
    // last iteration's traj
    Traj<3,3> _traj0;
    std::vector<double> _z0;

    void setZMPTraj(const Traj<3,3> &traj, const std::vector<double> &z0);
    double forwardPass(const double *x0, Traj<3,3> &com) const;
    void backwardPass(const Traj<3,3> &com);
    
    // x[6] is the z offset
    void getAB(const double x[6], const double u[3], double z0, Eigen::Matrix<double,6,6> &A, Eigen::Matrix<double,6,3> &B) const;
    Eigen::Vector3d computeAcc(const double x0[6], const double u[3], double z0) const;
    void integrate(const double x0[6], const double u[3], double z0, double acc[3], double x1[6]) const;

  public:
    LipmVarHeightPlanner();
    LipmVarHeightPlanner(double m, double dt, const Eigen::Matrix<double,6,6> &Q, const Eigen::Matrix<double,3,3> &R);
    ~LipmVarHeightPlanner();

    LipmVarHeightPlanner &operator= (const LipmVarHeightPlanner &other);
    bool getCOMTraj(const Traj<3,3> &zmp, const double *state, const std::vector<double> &z0, Traj<3,3> &com);
    
    Eigen::Vector3d getCOMAcc(size_t t, const Eigen::Vector3d &com, const Eigen::Vector3d &comd) const;
    void getCOMState(size_t t, Eigen::Vector3d &com, Eigen::Vector3d &comd) const;
}; 

#endif
