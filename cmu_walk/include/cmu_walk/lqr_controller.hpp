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

#ifndef LQR_CONTROLLER_H
#define LQR_CONTROLLER_H

#include <cassert>
#include <cstring>

#include <iostream>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Cholesky>
#include <eigen3/Eigen/Eigenvalues>

template <int xDim, int uDim, int infIter=5000>
class LqrController {
  private:
    Eigen::Matrix<double,xDim,1> _x;
    Eigen::Matrix<double,uDim,1> _u;

    Eigen::Matrix<double,xDim,xDim> _Q;
    Eigen::Matrix<double,uDim,uDim> _R;
    Eigen::Matrix<double,uDim,xDim> _K;
    Eigen::Matrix<double,xDim,xDim> _V;

    void dare(const Eigen::Matrix<double,xDim,xDim> &A, const Eigen::Matrix<double,xDim,uDim> &B, Eigen::Matrix<double,xDim,xDim> &P) 
    {
      Eigen::Matrix<double,xDim,xDim> Ainv = A.inverse();
      Eigen::Matrix<double,xDim,xDim> ABRB = Ainv * B * _R.llt().solve(B.transpose());
      
      Eigen::Matrix<double,2*xDim,2*xDim> Z;
      Z.block(0,0,xDim,xDim) = Ainv;
      Z.block(0,xDim,xDim,xDim) = ABRB;
      Z.block(xDim,0,xDim,xDim) = _Q * Ainv;
      Z.block(xDim,xDim,xDim,xDim) = A.transpose() + _Q * ABRB;

      Eigen::ComplexEigenSolver <Eigen::Matrix<double,2*xDim,2*xDim> > ces;
      ces.compute(Z);

      Eigen::Matrix<std::complex<double>,2*xDim,1> eigVal = ces.eigenvalues();
      Eigen::Matrix<std::complex<double>,2*xDim,2*xDim> eigVec = ces.eigenvectors();

      Eigen::Matrix<std::complex<double>,2*xDim,xDim> unstableEigVec;
      
      int ctr = 0;
      for (int i = 0; i < 2*xDim; i++) {
        if (eigVal(i).real()*eigVal(i).real() + eigVal(i).imag()*eigVal(i).imag() > 1) {
          unstableEigVec.col(ctr) = eigVec.col(i);
          ctr++;
          if (ctr > xDim)
            break;
        }
      }
      
      Eigen::Matrix<std::complex<double>,xDim,xDim> U21inv = unstableEigVec.block(0,0,xDim,xDim).inverse();
      Eigen::Matrix<std::complex<double>,xDim,xDim> PP = unstableEigVec.block(xDim,0,xDim,xDim) * U21inv;
      
      for (int i = 0; i < xDim; i++) {
        for (int j = 0; j < xDim; j++) {
          P(i,j) = PP(i,j).real();
        }
      }
    }

    void copy(const LqrController<xDim,uDim> & other) 
    {
      _x = other._x;  
      _u = other._u;
      _Q = other._Q;
      _R = other._R;
      _K = other._K;
      _V = other._V;
    }
    inline void computeU() { _u = _K*_x; }

  public:
    LqrController(const LqrController<xDim,uDim> & other) { copy(other); }
    LqrController<xDim,uDim> & operator=(const LqrController<xDim,uDim> & other) 
    {
      copy(other); 
      return *this;
    }

    LqrController() 
    {
      _Q.setIdentity();
      _R.setIdentity();
      reset();
    }
    ~LqrController() {}

    void reset()
    {
      _x.setZero();
      _u.setZero();
      _K.setZero();
      _V = _Q;
    }

    void LQRBackup(const Eigen::Matrix<double,xDim,xDim> & A, 
        const Eigen::Matrix<double,xDim,uDim> & B, 
        const Eigen::Matrix<double,xDim,xDim> & Q, 
        const Eigen::Matrix<double,uDim,uDim> & R)
    {
      setQ(Q);
      setR(R);
      LQRBackup(A, B);
    }

    void LQRBackup(const Eigen::Matrix<double,xDim,xDim> & A, 
        const Eigen::Matrix<double,xDim,uDim> & B)
    {
      Eigen::Matrix<double,uDim,xDim> tmpUX = B.transpose()*_V;
      _K = -(tmpUX*B + _R).llt().solve(tmpUX*A);

      Eigen::Matrix<double,xDim,xDim> tmpXX = A + B*_K;
      _V = tmpXX.transpose()*_V*tmpXX + _K.transpose()*_R*_K + _Q;
    }

    void infTimeLQR(const Eigen::Matrix<double,xDim,xDim> & A, 
        const Eigen::Matrix<double,xDim,uDim> & B)
    {
      reset();
      
      /*
      for (int i=0; i<infIter; i++) {
#if 0
        std::cout << "K: " << std::endl << _K; 
        << "V: " << std::endl << _V;
        << "===========================================" << std::endl;
#endif
        LQRBackup(A, B);
      }
      
      std::cout << "loop K\n" << _K << std::endl;
      std::cout << "loop V\n" << _V << std::endl;
      */

      dare(A, B, _V);
      _K = -(B.transpose() * _V * B + _R).llt().solve(B.transpose() * _V * A);
      
      //std::cout << "dare K\n" << _K << std::endl;
      //std::cout << "dare V\n" << _V << std::endl;
    }

    double oneStepScore() const
    {
      double scoreX = (_Q.transpose()*_x).dot(_x);
      double scoreU = (_R.transpose()*_u).dot(_u);
      return scoreX + scoreU;
    }

    double computeValue(const Eigen::Matrix<double,xDim,1> & x) const 
    {
      return (_V*x).dot(x);
    }

    void computeU(const double *x, int xLen, double *u, int uLen)
    {
      assert(xLen == xDim && uDim == uLen);
      setX(x, xLen);
      computeU();
      getU(u, uLen);
    }

    inline void computeU(const Eigen::Matrix<double,xDim,1> & x, 
        Eigen::Matrix<double,uDim,1> & u) 
    {
      _x = x;
      computeU();
      u = _u;
    }

    void setX(const double * x, int len) { 
      assert(len == xDim); 
      _x = Eigen::Map<const Eigen::Matrix<double,xDim,1> >(x);
    }

    //void setK(const Eigen::Matrix<double,uDim,xDim> & K) { _K = K; }
    void setQ(const Eigen::Matrix<double,xDim,xDim> & Q) { _Q = Q; }
    void setR(const Eigen::Matrix<double,uDim,uDim> & R) { _R = R; }

    void getU(double * u, int len) { 
      assert(len == uDim); 
      memcpy(u, _u.data(), sizeof(double)*len); 
    }

    const Eigen::Matrix<double,xDim,1> & getX() const { return _x; }
    const Eigen::Matrix<double,uDim,1> & getU() const { return _u; }

    const Eigen::Matrix<double,xDim,xDim> & getQ() const { return _Q; }
    const Eigen::Matrix<double,uDim,uDim> & getR() const { return _R; }
    const Eigen::Matrix<double,uDim,xDim> & getK() const { return _K; }
    const Eigen::Matrix<double,xDim,xDim> & getV() const { return _V; }
};

#endif

