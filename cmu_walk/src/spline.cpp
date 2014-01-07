/*
 * =====================================================================================
 *
 *       Filename:  spline.cpp
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  03/14/2013 03:33:30 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  YOUR NAME (), 
 *        Company:  
 *
 * =====================================================================================
 */

#include "spline.h"
#include <math.h>

int linear_spline( double the_time, 
    const knot_t *k0, 
    const knot_t *k1, 
    double *pos)
{
  if ( the_time <= k0->time )
  {
    if (pos)
      *pos = k0->pos;
    return 0;
  }
  if ( the_time >= k1->time )
  {
    if (pos)
      *pos = k1->pos;
    return 0;
  }
  if( fabs( (k1->time - k0->time) ) < 1e-5 )
  {
    if (pos)
      *pos = k1->pos;
    return 0;
  }

  double t = (the_time - k0->time) / (k1->time - k0->time);

  *pos = t*k1->pos + (1-t)*k0->pos;
}

int quintic_spline( double the_time, 
    const knot_t *k0, 
    const knot_t *k1, 
    double *pos, 
    double *vel, 
    double *acc )
{  
  double a, b, c, d, e, f;
  double tmp1, tmp2, tmp3;
  double t1, t2, t3, t4, t5;
  double nv0, nv1, na0, na1;

  if ( the_time <= k0->time )
  {
    if (pos)
      *pos = k0->pos;
    if (vel)
      *vel = k0->vel;
    if (acc)
      *acc = k0->acc;
    return 0;
  }
  if ( the_time >= k1->time )
  {
    if (pos)
      *pos = k1->pos;
    if (vel)
      *vel = k1->vel;
    if (acc)
      *acc = k1->acc;
    return 0;
  }
  if( fabs( (k1->time - k0->time) ) < 1e-5 )
  {
    if (pos)
      *pos = k1->pos;
    if (vel)
      *vel = k1->vel;
    if (acc)
      *acc = k1->acc;
    return 0;
  }
  nv0 = k0->vel*(k1->time - k0->time);
  nv1 = k1->vel*(k1->time - k0->time);
  na0 = k0->acc*(k1->time - k0->time)*(k1->time - k0->time);
  na1 = k1->acc*(k1->time - k0->time)*(k1->time - k0->time);

  f = k0->pos;
  e = nv0;
  d = na0/2;
  tmp1 = na0/2 + nv0 + k0->pos - k1->pos;
  tmp2 = na0 + nv0 - nv1;
  tmp3 = na0 - na1;
  a = - ( tmp3 - 6*tmp2 + 12*tmp1 )/2;
  b = - ( 8*a + 4*tmp2 - 12*tmp1 )/4;
  c = - ( a + b + tmp1 );
  t1 = (the_time - k0->time)/(k1->time - k0->time); 
  t2 = t1*t1;
  t3 = t2*t1;
  t4 = t3*t1;
  t5 = t4*t1;
  if (pos)
    *pos = a*t5 + b*t4 + c*t3 + d*t2 + e*t1 + f;
  if (vel)
    *vel = (5*a*t4 + 4*b*t3 + 3*c*t2 + 2*d*t1 + e)/(k1->time - k0->time);
  if (acc)
    *acc = (20*a*t3 + 12*b*t2 + 6*c*t1 + 2*d)/
    ((k1->time - k0->time)*(k1->time - k0->time));

  return 0;
}

void set_single_knot(double t0, double t1, double p0, double p1, knot_t *k0, knot_t *k1)
{
  k0->time = t0;
  k0->pos = p0;
  k0->vel = 0;
  k0->acc = 0;

  k1->time = t1;
  k1->pos = p1;
  k1->vel = 0;
  k1->acc = 0;
}
 
void set_foot_knots_pos_only(double t0, double t1, const double pos0[3], const double pos1[3], double z, knot_t *k0, knot_t *k1, knot_t *kz_mid)
{
  for (int i = 0; i < 3; i++) {
    k0[i].time = t0;
    k0[i].pos = pos0[i];
    k0[i].vel = 0;
    k0[i].acc = 0;
    k1[i].time = t1;
    k1[i].pos = pos1[i];
    k1[i].vel = 0;
    k1[i].acc = 0;
  }
  
  kz_mid->time = (t1+t0)/2.;
  kz_mid->pos = z;
  kz_mid->vel = 0;
  kz_mid->acc = 0;
}
 
void lookup_foot_pos_only(double t, const knot_t *k0, const knot_t *k1, const knot_t *kz_mid, 
  double *pos, double *vel, double *acc)
{
  for (int i = 0; i < 3; i++) {
    if (i != 2) {
      quintic_spline(t, k0+i, k1+i, pos+i, vel+i, acc+i);
      quintic_spline(t, k0+i, k1+i, pos+i, vel+i, acc+i);
    }
    else {
      if (t <= kz_mid->time)
        quintic_spline(t, k0+2, kz_mid, pos+2, vel+2, acc+2);
      else
        quintic_spline(t, kz_mid, k1+2, pos+2, vel+2, acc+2);    
    }
  }
}

void set_knots_pos_only(
  double t0, double t1, 
  const double pos0[3], const double pos1[3], 
  const double vel0[3], const double vel1[3],
  const double acc0[3], const double acc1[3], 
  knot_t k0[3], knot_t k1[3])
{
  for (int i = 0; i < 3; i++) {
    k0[i].time = t0;
    k0[i].pos = pos0[i];
    k0[i].vel = vel0[i];
    k0[i].acc = acc0[i];
    k1[i].time = t1;
    k1[i].pos = pos1[i];
    k1[i].vel = vel1[i];
    k1[i].acc = acc1[i];
  }
}

void lookup_pos_only(
  double t, const knot_t k0[3], const knot_t k1[3], 
  double pos[3], double vel[3], double acc[3])
{
  for (int i = 0; i < 3; i++) {
    quintic_spline(t, k0+i, k1+i, pos+i, vel+i, acc+i);
    quintic_spline(t, k0+i, k1+i, pos+i, vel+i, acc+i);
  }
}
 
