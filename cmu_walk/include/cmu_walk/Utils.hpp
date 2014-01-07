/*
 * =====================================================================================
 *
 *       Filename:  Utils.hpp
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  07/27/2013 03:38:16 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  YOUR NAME (), 
 *        Company:  
 *
 * =====================================================================================
 */

#ifndef __UTILS_HPP
#define __UTILS_HPP

#include <time.h>
#include <stdlib.h>
#include <fstream>
#include <string.h>
#include <math.h>

inline double limitMag(double &val, double maxMag) {
	if(val > maxMag)	val = maxMag;
	if(val < -maxMag)	val = -maxMag;
	return val;
}

inline void clamp(double &num, double min, double max) 
{
  num = (num > max) ? max : num;
  num = (num < min) ? min : num;
}

inline bool is_bad_num(double num) 
{ 
  return isinf(num) || isnan(num); 
}

inline double low_pass_filter(double &hist, double raw, double alpha)
{
  hist = hist * alpha + raw * (1-alpha);
  return hist;
}

inline void dvec_add(const double *a, const double *b, double *out, size_t n)
{
  for (size_t i = 0; i < n; i++)  
    out[i] = a[i] + b[i];
}

inline void dvec_sub(const double *a, const double *b, double *out, size_t n)
{
  for (size_t i = 0; i < n; i++)  
    out[i] = a[i] - b[i];
}

inline void dvec_set(double *dest, double num, size_t n)
{
  for (size_t i = 0; i < n; i++)  
    dest[i] = num;
}

inline void fvec_set(float *dest, float num, size_t n)
{
  for (size_t i = 0; i < n; i++)  
    dest[i] = num;
}

inline void dvec_copy(double *dest, const double *src, size_t n)
{ 
  memcpy(dest, src, sizeof(double)*n);
}

inline void fvec_copy(float *dest, const float *src, size_t n)
{ 
  memcpy(dest, src, sizeof(float)*n);
}

inline bool write_n_bin_double(const double *data, int len, std::ofstream &out)
{
  out.write((const char *)data, len*sizeof(double));
  return out.good();  
}

inline bool read_n_bin_double(double *data, int len, std::ifstream &in)
{
  in.read((char *)data, len*sizeof(double));
  return in.good();
}

inline double get_rand(double mag) { 
  return (((double)rand() / (double)RAND_MAX) - 0.5) * mag; 
}

inline void get_rand_vec(double *v, int len, double mag) {
  for (int i = 0; i < len; i++)  
    v[i] = get_rand(mag);
}

inline double get_time() {
  struct timespec the_tp;
  clock_gettime( CLOCK_MONOTONIC, &the_tp );
  return ((double) (the_tp.tv_sec)) + 1.0e-9*the_tp.tv_nsec;  
}

 
#endif
