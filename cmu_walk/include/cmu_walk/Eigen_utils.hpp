/*
 * =====================================================================================
 *
 *       Filename:  Eigen_utils.hpp
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  05/06/2013 12:31:09 AM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  YOUR NAME (), 
 *        Company:  
 *
 * =====================================================================================
 */

#ifndef __EIGEN_UTILS_HPP
#define __EIGEN_UTILS_HPP

#include <eigen3/Eigen/Geometry>
#include <stdio.h>
#include <string>

/*
// 2 rotations, first rotate world z to normal. then figure
// out how much to rotate around the new z, s.t. when we 
// project the resulting x axis to the world frame, it has
// the yaw angle we want.
// n123 are the normal, wtf is the angle for the second 
// rotation

// matlab code, 
syms n1 n2 n3 wtf
cross([0 0 1], [n1 n2 n3])
first_rot = sym_rot([-n2 n1 0]./sqrt(n1^2+n2^2), acos(n3))
second_rot = sym_rot([0 0 1], wtf);

rot = first_rot * second_rot;

x = rot(1,1);
y = rot(2,1);

ccode(x)
ccode(y)
*/ 



inline Eigen::Quaterniond normalAndYaw2Quat(Eigen::Vector3d normal, double yaw)
{
  if (normal.norm() == 0)
    return Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));

  normal.normalize();
  double n1 = normal(0);
  double n2 = normal(1);
  double n3 = normal(2);

  if (n1 == 0 && n2 == 0)
    return Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));

  double a1 = (((n1*n1)*(n3-1.0))/(n1*n1+n2*n2)+1.0);
  double a2 = (n1*n2*(n3-1.0))/(n1*n1+n2*n2);
  double a3 = (((n2*n2)*(n3-1.0))/(n1*n1+n2*n2)+1.0);
  double a4 = (n1*n2*(n3-1.0))/(n1*n1+n2*n2);

  double theta = atan2(a1*tan(yaw) - a4, a3 - a2*tan(yaw));

  //double x = cos(theta)*a1 + sin(theta)*a2;
  //double y = sin(theta)*a3 + cos(theta)*a4;
  
  Eigen::Vector3d firstAxis = (Eigen::Vector3d::UnitZ().cross(normal)).normalized();
  Eigen::AngleAxisd firstRot(acos(normal(2)), firstAxis);
  Eigen::AngleAxisd secondRot(theta, Eigen::Vector3d::UnitZ());

  return firstRot * secondRot;
}

inline Eigen::Vector3d transformVec(const Eigen::Vector3d &pos, const Eigen::Quaterniond &q, const Eigen::Vector3d &off)
{
  return pos + q.toRotationMatrix() * off;
}

inline Eigen::Vector2d rotate2d(const Eigen::Vector2d &in, double yaw)
{
  return Eigen::Vector2d(cos(yaw)*in[0] - sin(yaw)*in[1], sin(yaw)*in[0] + cos(yaw)*in[1]);
}

inline void neg_quat(Eigen::Quaterniond &q) 
{ 
  q.w() *= -1; 
  q.x() *= -1; 
  q.y() *= -1; 
  q.z() *= -1; 
}

inline Eigen::Vector3d getAxis(const Eigen::Quaterniond &q, int idx)
{
  Eigen::Matrix3d rot = q.toRotationMatrix();
  return rot.col(idx).normalized();
}

inline Eigen::Vector3d getAngVel(const Eigen::Quaterniond &quat0, Eigen::Quaterniond Qf, double T) {
	 if (Qf.dot(quat0) < 0)		neg_quat(Qf);

	 Eigen::AngleAxisd wQ = Eigen::AngleAxisd(Qf*(quat0.inverse()));
	 return  wQ.axis()*(wQ.angle()/T);
}

inline Eigen::Vector3d quatMinus(const Eigen::Quaterniond &Qm, Eigen::Quaterniond Qd) {
  if (Qm.dot(Qd) < 0)	    
    neg_quat(Qd);
  Eigen::AngleAxisd rotVec(Qm * (Qd.inverse().normalized()));
  return rotVec.axis() * rotVec.angle();
}

inline double quatDist(const Eigen::Quaterniond &a, const Eigen::Quaterniond &b)
{
  Eigen::Quaterniond tmp = a;
  if (tmp.dot(b) < 0)
    neg_quat(tmp);

  Eigen::AngleAxisd rotVec(tmp * (b.inverse()));
  return rotVec.angle(); 
}

inline Eigen::Quaterniond euler2quat(const Eigen::Vector3d &ang)
{
  Eigen::Matrix3d rot;
  rot = 
    Eigen::AngleAxisd(ang[0], Eigen::Vector3d::UnitX()) *
    Eigen::AngleAxisd(ang[1], Eigen::Vector3d::UnitY()) * 
    Eigen::AngleAxisd(ang[2], Eigen::Vector3d::UnitZ()); 
  return Eigen::Quaterniond(rot).normalized();
}

inline Eigen::Quaterniond zyx2quat(const Eigen::Vector3d &ang)
{
  Eigen::Matrix3d rot;
  rot = 
    Eigen::AngleAxisd(ang[2], Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(ang[1], Eigen::Vector3d::UnitY()) * 
    Eigen::AngleAxisd(ang[0], Eigen::Vector3d::UnitX()); 
  return Eigen::Quaterniond(rot).normalized();
}

inline Eigen::Vector3d quat2zyx(const Eigen::Quaterniond &q)
{
  double q0 = q.w();
  double q1 = q.x();
  double q2 = q.y();
  double q3 = q.z();

  double phi = atan2(2*(q0*q1+q2*q3), 1-2*(q1*q1+q2*q2));
  double theta = asin(2*(q0*q2-q3*q1));
  double psi = atan2(2*(q0*q3+q1*q2), 1-2*(q2*q2+q3*q3));
  return Eigen::Vector3d(phi, theta, psi);
}

inline Eigen::Quaterniond onlyYaw(const Eigen::Quaterniond &q)
{
  Eigen::Vector3d zyx = quat2zyx(q);
  zyx[0] = zyx[1] = 0;
  return zyx2quat(zyx);
}

inline double getYaw(const Eigen::Quaterniond &q)
{
  return quat2zyx(q).z();  
}

inline void printQuat(const std::string &name, const Eigen::Quaterniond &q) 
{
  printf("%s, qx %5g qy %5g qz %5g qw %5g\n", name.c_str(), q.x(), q.y(), q.z(), q.w());
}

inline double avgAngle(double a, double b) 
{
  double x = fmod(fabs(a-b), 2*M_PI);
  if (x >= 0 && x <= M_PI)
    return fmod((a + b) / 2., 2*M_PI);
  else if (x >= M_PI && x < 1.5*M_PI)
    return fmod((a + b) / 2., 2*M_PI) + M_PI;
  else
    return fmod((a + b) / 2., 2*M_PI) - M_PI;
}

inline void quat2EA(const Eigen::Quaterniond &q, double *EA) {
  double Rtmp[3][3];
  double e1,e2,e3,e4,e11,e22,e33,e44,norm;
  double ie1 = q.x();
  double ie2 = q.y();
  double ie3 = q.z();
  double ie4 = q.w();

  e11 = ie1*ie1;
  e22 = ie2*ie2;
  e33 = ie3*ie3;
  e44 = ie4*ie4;
  norm = sqrt(e11+e22+e33+e44);
  if (norm == 0.) {
    e4 = 1.;
    norm = 1.;
  } else {
    e4 = ie4;
  }
  norm = 1./norm;
  e1 = ie1*norm;
  e2 = ie2*norm;
  e3 = ie3*norm;
  e4 = e4*norm;
  e11 = e1*e1;
  e22 = e2*e2;
  e33 = e3*e3;
  Rtmp[0][0] = 1.-(2.*(e22+e33));
  Rtmp[0][1] = 2.*(e1*e2-e3*e4);
  Rtmp[0][2] = 2.*(e1*e3+e2*e4);
  Rtmp[1][0] = 2.*(e1*e2+e3*e4);
  Rtmp[1][1] = 1.-(2.*(e11+e33));
  Rtmp[1][2] = 2.*(e2*e3-e1*e4);
  Rtmp[2][0] = 2.*(e1*e3-e2*e4);
  Rtmp[2][1] = 2.*(e2*e3+e1*e4);
  Rtmp[2][2] = 1.-(2.*(e11+e22));

  double th1,th2,th3, tmp;

  if (((fabs(Rtmp[0][2])-1.) >= -1e-15)  ) {
    th1 = atan2(Rtmp[2][1],Rtmp[1][1]);
    if ((Rtmp[0][2] > 0.)  ) {
      tmp = 1.5707963267949;
    } else {
      tmp = -1.5707963267949;
    }
    th2 = tmp;
    th3 = 0.;
  } else {
    th1 = atan2(-Rtmp[1][2],Rtmp[2][2]);
    th2 = asin(Rtmp[0][2]);
    th3 = atan2(-Rtmp[0][1],Rtmp[0][0]);
  }
  EA[0] = th1;
  EA[1] = th2;
  EA[2] = th3;
}

inline Eigen::Quaterniond EA2quat(const double *EA) {
  double cos1,cos2,cos3,sin1,sin2,sin3;
  double Rtmp[3][3];

  cos1 = cos(EA[0]);
  cos2 = cos(EA[1]);
  cos3 = cos(EA[2]);
  sin1 = sin(EA[0]);
  sin2 = sin(EA[1]);
  sin3 = sin(EA[2]);
  Rtmp[0][0] = (cos2*cos3);
  Rtmp[0][1] = -(cos2*sin3);
  Rtmp[0][2] = sin2;
  Rtmp[1][0] = ((cos1*sin3)+(sin1*(cos3*sin2)));
  Rtmp[1][1] = ((cos1*cos3)-(sin1*(sin2*sin3)));
  Rtmp[1][2] = -(cos2*sin1);
  Rtmp[2][0] = ((sin1*sin3)-(cos1*(cos3*sin2)));
  Rtmp[2][1] = ((cos1*(sin2*sin3))+(cos3*sin1));
  Rtmp[2][2] = (cos1*cos2);

  double tmp,tmp1,tmp2,tmp3,tmp4;

  tmp = (Rtmp[0][0]+(Rtmp[1][1]+Rtmp[2][2]));
  if (((tmp >= Rtmp[0][0]) && ((tmp >= Rtmp[1][1]) && (tmp >= Rtmp[2][2]
            )))  ) {
    tmp1 = (Rtmp[2][1]-Rtmp[1][2]);
    tmp2 = (Rtmp[0][2]-Rtmp[2][0]);
    tmp3 = (Rtmp[1][0]-Rtmp[0][1]);
    tmp4 = (1.+tmp);
  } else {
    if (((Rtmp[0][0] >= Rtmp[1][1]) && (Rtmp[0][0] >= Rtmp[2][2]))
       ) {
      tmp1 = (1.-(tmp-(2.*Rtmp[0][0])));
      tmp2 = (Rtmp[0][1]+Rtmp[1][0]);
      tmp3 = (Rtmp[0][2]+Rtmp[2][0]);
      tmp4 = (Rtmp[2][1]-Rtmp[1][2]);
    } else {
      if ((Rtmp[1][1] >= Rtmp[2][2])  ) {
        tmp1 = (Rtmp[0][1]+Rtmp[1][0]);
        tmp2 = (1.-(tmp-(2.*Rtmp[1][1])));
        tmp3 = (Rtmp[1][2]+Rtmp[2][1]);
        tmp4 = (Rtmp[0][2]-Rtmp[2][0]);
      } else {
        tmp1 = (Rtmp[0][2]+Rtmp[2][0]);
        tmp2 = (Rtmp[1][2]+Rtmp[2][1]);
        tmp3 = (1.-(tmp-(2.*Rtmp[2][2])));
        tmp4 = (Rtmp[1][0]-Rtmp[0][1]);
      }
    }
  }
  tmp = (1./sqrt(((tmp1*tmp1)+((tmp2*tmp2)+((tmp3*tmp3)+(tmp4*tmp4))))));
  return Eigen::Quaterniond((tmp*tmp4), (tmp*tmp1), (tmp*tmp2), (tmp*tmp3));
}

inline Eigen::Quaterniond EA2quat(double r, double p, double y) {
  double EA[3] = {r, p, y};
  return EA2quat(EA);
}

inline void flattenQuat(Eigen::Quaterniond &q) {
  double EA[3];
  quat2EA(q, EA);
  EA[0] = EA[1] = 0.0;
  q = EA2quat(EA);
}

inline Eigen::Quaterniond flatQuat(const Eigen::Quaterniond &q) {
  double EA[3];
  quat2EA(q, EA);
  EA[0] = EA[1] = 0.0;
  return EA2quat(EA);
}

//frac = 0: base
//frac = 1: target
inline Eigen::Quaterniond mySlerp(Eigen::Quaterniond &base, Eigen::Quaterniond target, double frac) {
  if (base.dot(target) < 0)	    neg_quat(target);	//go the short direction
  return base.slerp(frac, target);
}

inline void modifyQ(Eigen::Quaterniond &q, double r, double p, double y) {
  double EA[3] = {r,p,y};
  Eigen::Quaterniond q2 = EA2quat(EA);
  q=q*q2;
}

inline void preModifyQ(Eigen::Quaterniond &q, double r, double p, double y) {
  double EA[3] = {r,p,y};
  Eigen::Quaterniond q2 = EA2quat(EA);
  q=q2*q;
}

inline Eigen::Matrix3d getBasis(Eigen::Vector3d v0) {
  v0.normalize();
  Eigen::Vector3d zhat(0,0,1);
  Eigen::Vector3d v1 = zhat.cross(v0);
  Eigen::Vector3d v2 = v0.cross(v1);

  Eigen::Matrix3d m;
  m.block<1, 3>(0,0) = v0;	//top row is original vector
  m.block<1, 3>(1,0) = v1;
  m.block<1, 3>(2,0) = v2;

  return m;
}

inline void yawQuat(Eigen::Quaterniond &q, double yaw) {
	Eigen::Quaterniond dR = EA2quat(0,0,yaw);
	q = dR*q;
}


#endif
