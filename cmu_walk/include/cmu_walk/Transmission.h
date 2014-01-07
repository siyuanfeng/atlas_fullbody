/*
 * =====================================================================================
 *
 *       Filename:  Transmission.h
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  10/24/2013 04:10:52 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  YOUR NAME (), 
 *        Company:  
 *
 * =====================================================================================
 */

#ifndef __ATLAS_TRANSMISSION_TABLE_H
#define __ATLAS_TRANSMISSION_TABLE_H

#include <cmu_walk/drc_common_defines.h>

#define BACK_BKY_TABLE_LENGTH 29
#define BACK_BKX_TABLE_LENGTH 30

#define L_LEG_HPZ_TABLE_LENGTH 30
#define L_LEG_HPX_TABLE_LENGTH 30
#define L_LEG_HPY_TABLE_LENGTH 30
#define L_LEG_KNY_TABLE_LENGTH 30

#define R_LEG_HPZ_TABLE_LENGTH 30
#define R_LEG_HPX_TABLE_LENGTH 30
#define R_LEG_HPY_TABLE_LENGTH 30
#define R_LEG_KNY_TABLE_LENGTH 30

class Transmission {
  private:
    // in^2
    static const double AreaPos[N_JOINTS];
    static const double AreaNeg[N_JOINTS];
    static const double MaxTorquePos[N_JOINTS];
    static const double MaxTorqueNeg[N_JOINTS];

    static const double back_bky_table[BACK_BKY_TABLE_LENGTH][3];
    static const double back_bkx_table[BACK_BKX_TABLE_LENGTH][3];

    static const double l_leg_hpz_table[L_LEG_HPZ_TABLE_LENGTH][3];
    static const double l_leg_hpx_table[L_LEG_HPX_TABLE_LENGTH][3];
    static const double l_leg_hpy_table[L_LEG_HPY_TABLE_LENGTH][3];
    static const double l_leg_kny_table[L_LEG_KNY_TABLE_LENGTH][3];
    
    static const double r_leg_hpz_table[R_LEG_HPZ_TABLE_LENGTH][3];
    static const double r_leg_hpx_table[R_LEG_HPX_TABLE_LENGTH][3];
    static const double r_leg_hpy_table[R_LEG_HPY_TABLE_LENGTH][3];
    static const double r_leg_kny_table[R_LEG_KNY_TABLE_LENGTH][3];
  
    static const void *getTable(int i);

    static int findCell(const double table[][3], int start, int end, double angle);
    
  public:
    static bool isAcuatorSide(int j);

    static int getTableLen(int j);
    static int findCell(const double table[][3], int length, double angle) { return findCell(table, 0, length-1, angle); }

    static double getMomentArm(int j, double ang);
    static void getTorqueLimit(int j, double ang, double *min_torque, double *max_torque);
   
    // call find cell first
};

#endif
