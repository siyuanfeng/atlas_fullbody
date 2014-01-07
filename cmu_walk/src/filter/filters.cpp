/*
 * =====================================================================================
 *
 *       Filename:  filters.cpp
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  05/07/2013 10:04:09 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  YOUR NAME (), 
 *        Company:  
 *
 * =====================================================================================
 */

//#include "filter/filters.h"
#include <eigen3/Eigen/Geometry>
#include <stdio.h>

/************************************************************************************************/ 
// Compute Cross Product Matrix of a vector
/************************************************************************************************/  
void crossProductMatrix(const Eigen::Matrix<double,3,1> & u, Eigen::Matrix<double,3,3> & X)
{
  X.setZero();
  X(0,1) = -u(2,0);
  X(1,0) = u(2,0);
  X(0,2) = u(1,0);
  X(2,0) = -u(1,0);
  X(1,2) = -u(0,0);
  X(2,1) = u(0,0);
 //     0, -u(2), u(1),
 //     u(2), 0, -u(0),
 //     -u(1), u(0), 0;  
}       

/************************************************************************************************/ 
// Normalize quaternion
/************************************************************************************************/  
void fixQuaternion(Eigen::Quaternion<double> & q)
{
  if (fabs(q.norm()-1.0) > 1e-2) 
    printf("Quaternion norm is %g, renormalize\n", q.norm());
  q.normalize(); // Normalize   
  // if (q.w() < 0) q.coeffs() *= -1.0; // Flip
}   
 
