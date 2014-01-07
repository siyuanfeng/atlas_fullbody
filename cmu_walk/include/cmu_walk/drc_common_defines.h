/*
 * =====================================================================================
 *
 *       Filename:  sim_common.h
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  01/14/2013 04:34:30 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  YOUR NAME (), 
 *        Company:  
 *
 * =====================================================================================
 */

#ifndef __SIM_COMMON_H
#define __SIM_COMMON_H

enum ContactState {
	DSc = 0,
	SSL,
	SSR,
	DSl,  // cop shifting to left (right lift off)
	DSr,  // cop shifting to right (left lift off)
	FLYING
};

#define MAX_N_SDFAST_U    34

#define OK      0
#define CRASHED 1 

#define LEFT 0
#define RIGHT 1
#define LR 2

#define XX 0
#define YY 1
#define ZZ 2
#define XYZ 3

#define RPY 3

// Quaternion. Q3 is cos(a/2), Q0-2 is axis*sin(a/2)
#define Q0 0
#define Q1 1
#define Q2 2
#define Q3 3
#define QQ 4

// some constants
#define GRAVITY     9.81
#define FRICTION    1

#define A_BACK_LBZ  0 // back
#define A_BACK_MBY  1 
#define A_BACK_UBX  2
#define A_NECK_AY   3 // neck fe
#define A_L_LEG_UHZ 4
#define A_L_LEG_MHX 5
#define A_L_LEG_LHY 6
#define A_L_LEG_KNY 7
#define A_L_LEG_UAY 8
#define A_L_LEG_LAX 9
#define A_R_LEG_UHZ 10
#define A_R_LEG_MHX 11
#define A_R_LEG_LHY 12
#define A_R_LEG_KNY 13
#define A_R_LEG_UAY 14
#define A_R_LEG_LAX 15
#define A_L_ARM_USY 16 // weird joint, axis = 0.000000 -0.500000 0.866025
#define A_L_ARM_SHX 17
#define A_L_ARM_ELY 18
#define A_L_ARM_ELX 19
#define A_L_ARM_UWY 20 // wrist
#define A_L_ARM_MWX 21
#define A_R_ARM_USY 22 // weird joint, axis = 0.000000 0.500000 0.866025
#define A_R_ARM_SHX 23
#define A_R_ARM_ELY 24
#define A_R_ARM_ELX 25
#define A_R_ARM_UWY 26 // wrist
#define A_R_ARM_MWX 27   

#define N_BODIES    29
#define N_JOINTS    28
#define N_LOCATIONS 28 

#define L_F0_J0 28
#define L_F0_J1 29
#define L_F0_J2 30

#define L_F1_J0 31
#define L_F1_J1 32
#define L_F1_J2 33

#define L_F2_J0 34
#define L_F2_J1 35
#define L_F2_J2 36

#define L_F3_J0 37
#define L_F3_J1 38
#define L_F3_J2 39

#define R_F0_J0 40
#define R_F0_J1 41
#define R_F0_J2 42

#define R_F1_J0 43
#define R_F1_J1 44
#define R_F1_J2 45

#define R_F2_J0 46
#define R_F2_J1 47
#define R_F2_J2 48

#define R_F3_J0 49
#define R_F3_J1 50
#define R_F3_J2 51

#define N_HAND_JOINTS 12 


inline bool isDS(int state) {
  return (state == DSc || state == DSl || state == DSr);
}

inline bool isSwing(int cState, int side) {
  switch (side) {
    case LEFT:
      return cState != SSL && !isDS(cState);
    case RIGHT:
      return cState != SSR && !isDS(cState);
    default:
      return false;
  }
}

inline bool isToeOff(int cState, int side) {
  switch (side) {
    case LEFT:
      return cState == DSr;
    case RIGHT:
      return cState == DSl;
    default:
      return false;
  }
}

#endif
