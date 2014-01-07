/*
 * =====================================================================================
 *
 *       Filename:  spline.h
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  03/14/2013 03:34:05 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  YOUR NAME (), 
 *        Company:  
 *
 * =====================================================================================
 */

#ifndef __SPLINE_H
#define __SPLINE_H

typedef struct {
  double time;
  double pos;
  double vel;
  double acc; 
} knot_t;

int quintic_spline(double the_time, 
    const knot_t *k0, 
    const knot_t *k1, 
    double *pos, 
    double *vel, 
    double *acc);

int linear_spline(double time, const knot_t *k0, const knot_t *k1, double *pos);

void set_single_knot(double t0, double t1, double yaw0, double yaw1, knot_t *k0, knot_t *k1);
void set_foot_knots_pos_only(double t0, double t1, const double pos0[3], const double pos1[3], double z, knot_t *k0, knot_t *k1, knot_t *kz_mid);
void lookup_foot_pos_only(double t, const knot_t *k0, const knot_t *k1, const knot_t *kz_mid, double *pos, double *vel, double *acc);
void set_knots_pos_only(double t0, double t1, const double pos0[3], const double pos1[3], const double vel0[3], const double vel1[3],const double ac0[3], const double acc1[3], knot_t k0[3], knot_t k1[3]);
void lookup_pos_only(double t, const knot_t k0[3], const knot_t k1[3], double pos[3], double vel[3], double acc[3]);

#endif
