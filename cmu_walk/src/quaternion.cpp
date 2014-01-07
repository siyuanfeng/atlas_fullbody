/*
 * =====================================================================================
 *
 *       Filename:  quaternion.c
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  03/22/2013 10:44:18 AM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  YOUR NAME (), 
 *        Company:  
 *
 * =====================================================================================
 */

#include "quaternion.h"
#include <math.h>
#include <stdlib.h>

/*
Quaternion::Quaternion()
{
  _q[0] = _q[1] = _q[2] = 0;
  _q[3] = 1;
}

Quaternion Quaternion::slerp(const Quaternion &q0, const Quaternion &q1, double t)
{
  
}

void Quaternion::toEuler(double ang[3]) const
{
  double dircos[3][3];
  quat2dc(q, dircos);
  dc2ang((const double (*)[3])dircos, ang);  
}

    Quaternion negate() const;
    double quat_dot(const Quaternion &q1) const;
    Quaternion invert() const;
    Quaternion compose(const Quaternion &p) const;

    double normalize_q();
    void q_to_rotvec(double *ang, double rotvec[3]) const;  
    void transform_vec(const double pos[3], const double off[3], double out[3]) const;
    void q_to_r(double r[3][3]) const;

    // sdfast dc quat euler angle stuff
    void dc2ang(const double dircos[3][3], double ang[3]);
    void dc2quat(const double dircos[3][3], double q[4]);
    void ang2dc(const double ang[3], double dircos[3][3]);
    void quat2dc(const double q[4], double dircos[3][3]);

void slerp(const double q0[4], const double q1[4], double t, double q_out[4])
{ 
  int i;
  double cosHalfTheta = quat_dot(q0, q1);
  double q_fin[4];
  if (t > 1)
    t = 1;
  if (t < 0)
    t = 0;

  if (cosHalfTheta < 0) {
    for (i = 0; i < 4; i++)
      q_fin[i] = -q1[i];
    cosHalfTheta = -cosHalfTheta;
  }
  else {
    for (i = 0; i < 4; i++)
      q_fin[i] = q1[i];
  }

  if (fabs(cosHalfTheta) > 0.9995) {
    for (i = 0; i < 4; i++)
      q_out[i] = q0[i];
    return;
  }
  
  double halfTheta = acos(cosHalfTheta);
  double sinHalfTheta = sqrt(1.0 - cosHalfTheta*cosHalfTheta);
  // if theta = 180 degrees then result is not fully defined
  // we could rotate around any axis normal to qa or qb
  if (fabs(sinHalfTheta) < 0.001){ // fabs is floating point absolute
    for (i = 0; i < 4; i++)
      q_out[i] = (q0[i] * 0.5 + q_fin[i] * 0.5);
    return;
  }

  double ratioA = sin((1 - t) * halfTheta) / sinHalfTheta;
  double ratioB = sin(t * halfTheta) / sinHalfTheta; 
  for (i = 0; i < 4; i++)
    q_out[i] = q0[i] * ratioA + q_fin[i] * ratioB;

  normalize_q(q_out);
}
*/

void neg_q(double q[4])
{
  int i;
  for (i = 0; i < 4; i++)
    q[i] = -q[i];
}

double quat_dot( const double q0[4], const double q1[4] )
{
  int i;
  double res = 0;
  for (i = 0; i < 4; i++)
    res += q0[i]*q1[i];
  return res;
}

void invert_q( const double *q_in, double *q_out )
{
  int i;
  for ( i = 0; i < 3; i++ )
    q_out[i] = -q_in[i];
  q_out[3] = q_in[3];
}

/************************************************/
// make the quaternion length = 1.                                              
double normalize_q( double *q )
{
  int i;
  double length_2 = 0;
  double length;
  double scale;

  for ( i = 0; i < 4; i++ )
    length_2 += q[i]*q[i];
  length = sqrt( length_2 );
  scale = 1.0/length;
  for ( i = 0; i < 4; i++ )
    q[i] *= scale;
  return length;
}

/************************************************/
// res = q * p, I think
void compose_q( const double *q, const double *p, double *result )
{
  /*
     result[0] = q[0]*p[0] - q[1]*p[1] - q[2]*p[2] - q[3]*p[3];
     result[1] = q[0]*p[1] + q[1]*p[0] + q[2]*p[3] - q[3]*p[2];
     result[2] = q[0]*p[2] + q[2]*p[0] + q[3]*p[1] - q[1]*p[3];
     result[3] = q[0]*p[3] + q[3]*p[0] + q[1]*p[2] - q[2]*p[1];
     */
  result[0] = q[3]*p[0] + q[0]*p[3] + q[1]*p[2] - q[2]*p[1];
  result[1] = q[3]*p[1] + q[1]*p[3] + q[2]*p[0] - q[0]*p[2];
  result[2] = q[3]*p[2] + q[2]*p[3] + q[0]*p[1] - q[1]*p[0];
  result[3] = q[3]*p[3] - q[0]*p[0] - q[1]*p[1] - q[2]*p[2];
}

void q_to_r(const double *q, double r[3][3])
{
  r[0][0] = 2*(q[3]*q[3] + q[0]*q[0]) - 1.0;
  r[0][1] = 2*(q[0]*q[1] - q[3]*q[2]);
  r[0][2] = 2*(q[0]*q[2] + q[3]*q[1]);
  r[1][0] = 2*(q[0]*q[1] + q[3]*q[2]);
  r[1][1] = 2*(q[3]*q[3] + q[1]*q[1]) - 1.0;
  r[1][2] = 2*(q[1]*q[2] - q[3]*q[0]);
  r[2][0] = 2*(q[0]*q[2] - q[3]*q[1]);
  r[2][1] = 2*(q[1]*q[2] + q[3]*q[0]);
  r[2][2] = 2*(q[3]*q[3] + q[2]*q[2]) - 1.0;
}

/************************************************/
// Create a 3 vector that is the axis of rotation times the angle

void q_to_rotvec( const double *q, double *ang, double *rotvec )
{
  int i;
  double s;

  s = sqrt( q[0]*q[0] + q[1]*q[1] + q[2]*q[2] );
  *ang = 2*atan2( s, q[3] );
  //*ang = 2*acos(q[3]);
  if ( fabs( s ) > 1e-7 )
  {
    for ( i = 0; i < 3; i++ )
      rotvec[i] = (*ang)*q[i]/s;
  }
  else
  {
    for ( i = 0; i < 3; i++ )
      rotvec[i] = 0.0;
  }
}

void transform_vec(const double pos[3], const double q[4], const double off[3], double out[3])
{
  // first 3 are weld to ankle offset
  double v[4] = {off[0], off[1], off[2], 0};
  double q_inv[4];
  double tmp[4], res[4] = {0};
  int i;

  if (off[0]*off[0] + off[1]*off[1] + off[2]*off[2] >= 1e-15) {
    // rotate offset
    invert_q(q, q_inv);
    normalize_q(q_inv);
    compose_q(q, v, tmp);
    compose_q(tmp, q_inv, res);
  }

  for (i = 0; i < 3; i++) {
    if (pos)
      out[i] = pos[i] + res[i];
    else
      out[i] = res[i];
  }
}

void q_to_euler(const double q[4], double ang[3])
{
  double dircos[3][3];
  quat2dc(q, dircos);
  dc2ang((const double (*)[3])dircos, ang);
}

void euler_to_q(const double ang[3], double q[4])
{
  double dircos[3][3];
  ang2dc(ang, dircos);
  dc2quat((const double (*)[3])dircos, q);
}

/////////////////////////////////////////////////////////////////////////
// sdfast functions 
void dc2ang(const double dircos[3][3], double ang[3])
{
  double th1,th2,th3,local_temp[10];

  if (((fabs(dircos[0][2])-1.) >= -1e-15)  ) {
    th1 = atan2(dircos[2][1],dircos[1][1]);
    if ((dircos[0][2] > 0.)  ) {
      local_temp[0] = 1.5707963267949;
    } else {
      local_temp[0] = -1.5707963267949;
    }
    th2 = local_temp[0];
    th3 = 0.;
  } else {
    th1 = atan2(-dircos[1][2],dircos[2][2]);
    th2 = asin(dircos[0][2]);
    th3 = atan2(-dircos[0][1],dircos[0][0]);
  }
  ang[0] = th1;
  ang[1] = th2;
  ang[2] = th3;
}

void dc2quat(const double dircos[3][3], double q[4])
{
  double tmp,tmp1,tmp2,tmp3,tmp4,local_temp[10];

  tmp = (dircos[0][0]+(dircos[1][1]+dircos[2][2]));
  if (((tmp >= dircos[0][0]) && ((tmp >= dircos[1][1]) && (tmp >= dircos[2][2]
            )))  ) {
    tmp1 = (dircos[2][1]-dircos[1][2]);
    tmp2 = (dircos[0][2]-dircos[2][0]);
    tmp3 = (dircos[1][0]-dircos[0][1]);
    tmp4 = (1.+tmp);
  } else {
    if (((dircos[0][0] >= dircos[1][1]) && (dircos[0][0] >= dircos[2][2]))  
       ) {
      tmp1 = (1.-(tmp-(2.*dircos[0][0])));
      tmp2 = (dircos[0][1]+dircos[1][0]);
      tmp3 = (dircos[0][2]+dircos[2][0]);
      tmp4 = (dircos[2][1]-dircos[1][2]);
    } else {
      if ((dircos[1][1] >= dircos[2][2])  ) {
        tmp1 = (dircos[0][1]+dircos[1][0]);
        tmp2 = (1.-(tmp-(2.*dircos[1][1])));
        tmp3 = (dircos[1][2]+dircos[2][1]);
        tmp4 = (dircos[0][2]-dircos[2][0]);
      } else {
        tmp1 = (dircos[0][2]+dircos[2][0]);
        tmp2 = (dircos[1][2]+dircos[2][1]);
        tmp3 = (1.-(tmp-(2.*dircos[2][2])));
        tmp4 = (dircos[1][0]-dircos[0][1]);
      }
    }
  }
  tmp = (1./sqrt(((tmp1*tmp1)+((tmp2*tmp2)+((tmp3*tmp3)+(tmp4*tmp4))))));
  q[0] = (tmp*tmp1);
  q[1] = (tmp*tmp2);
  q[2] = (tmp*tmp3);
  q[3] = (tmp*tmp4);
}

void ang2dc(const double ang[3], double dircos[3][3])
{
  double cos1,cos2,cos3,sin1,sin2,sin3;

  cos1 = cos(ang[0]);
  cos2 = cos(ang[1]);
  cos3 = cos(ang[2]);
  sin1 = sin(ang[0]);
  sin2 = sin(ang[1]);
  sin3 = sin(ang[2]);
  dircos[0][0] = (cos2*cos3);
  dircos[0][1] = -(cos2*sin3);
  dircos[0][2] = sin2;
  dircos[1][0] = ((cos1*sin3)+(sin1*(cos3*sin2)));
  dircos[1][1] = ((cos1*cos3)-(sin1*(sin2*sin3)));
  dircos[1][2] = -(cos2*sin1);
  dircos[2][0] = ((sin1*sin3)-(cos1*(cos3*sin2)));
  dircos[2][1] = ((cos1*(sin2*sin3))+(cos3*sin1));
  dircos[2][2] = (cos1*cos2);
}

void quat2dc(const double q[4], double dircos[3][3])
{
  double e1,e2,e3,e4,e11,e22,e33,e44,norm;

  e11 = q[0]*q[0];
  e22 = q[1]*q[1];
  e33 = q[2]*q[2];
  e44 = q[3]*q[3];
  norm = sqrt(e11+e22+e33+e44);
  if (norm == 0.) {
    e4 = 1.;
    norm = 1.;
  } else {
    e4 = q[3];
  }
  norm = 1./norm;
  e1 = q[0]*norm;
  e2 = q[1]*norm;
  e3 = q[2]*norm;
  e4 = e4*norm;
  e11 = e1*e1;
  e22 = e2*e2;
  e33 = e3*e3;
  dircos[0][0] = 1.-(2.*(e22+e33));
  dircos[0][1] = 2.*(e1*e2-e3*e4);
  dircos[0][2] = 2.*(e1*e3+e2*e4);
  dircos[1][0] = 2.*(e1*e2+e3*e4);
  dircos[1][1] = 1.-(2.*(e11+e33));
  dircos[1][2] = 2.*(e2*e3-e1*e4);
  dircos[2][0] = 2.*(e1*e3-e2*e4);
  dircos[2][1] = 2.*(e2*e3+e1*e4);
  dircos[2][2] = 1.-(2.*(e11+e22));
}
 
