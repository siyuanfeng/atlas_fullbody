/*
 * =====================================================================================
 *
 *       Filename:  quaternion.h
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  03/22/2013 10:45:09 AM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  YOUR NAME (), 
 *        Company:  
 *
 * =====================================================================================
 */

#ifndef __QUATERNION_H
#define __QUATERNION_H

void slerp(const double q0[4], const double q1[4], double t, double q_out[4]);

void q_to_euler(const double q[4], double ang[3]);
void euler_to_q(const double ang[3], double q[4]);

void neg_q(double q[4]);
double quat_dot( const double q0[4], const double q1[4] );
void invert_q( const double q_in[4], double q_out[4] );
void compose_q( const double q[4], const double p[4], double result[4] );
double normalize_q( double q[4] );
void q_to_rotvec( const double q[4], double *ang, double rotvec[3] );  
void transform_vec(const double pos[3], const double q[4], const double off[3], double out[3]);
void q_to_r(const double *q, double r[3][3]);

// sdfast dc quat euler angle stuff
void dc2ang(const double dircos[3][3], double ang[3]);
void dc2quat(const double dircos[3][3], double q[4]);
void ang2dc(const double ang[3], double dircos[3][3]);
void quat2dc(const double q[4], double dircos[3][3]);

/*
class Quaternion {
  private:
    double _q[4];

  public:
    Quaternion();
    inline double x() const {return _q[0];}
    inline double y() const {return _q[1];}
    inline double z() const {return _q[2];}
    inline double w() const {return _q[3];}

    static Quaternion slerp(const Quaternion &q0, const Quaternion &q1, double t);
    void toEuler(double ang[3]) const;
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
};
*/

#endif
