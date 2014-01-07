#ifndef __DRC_PELVIS_DEFINES
#define __DRC_PELVIS_DEFINES

#define PELV_N_SDFAST_Q     35
#define PELV_N_SDFAST_U     34
#define PELV_N_SDFAST_STATE (PELV_N_SDFAST_U+PELV_N_SDFAST_Q)

/* Indices for sdfast state */
#define PELV_S_X     0
#define PELV_S_Y     1
#define PELV_S_Z     2
#define PELV_S_Q0    3
#define PELV_S_Q1    4
#define PELV_S_Q2    5
#define PELV_S_Q3    34

#define PELV_S_BACK_LBZ   6 // back
#define PELV_S_BACK_MBY   7 
#define PELV_S_BACK_UBX   8
#define PELV_S_NECK_AY   9 // neck fe
#define PELV_S_L_LEG_UHZ 10
#define PELV_S_L_LEG_MHX 11
#define PELV_S_L_LEG_LHY 12
#define PELV_S_L_LEG_KNY 13
#define PELV_S_L_LEG_UAY 14
#define PELV_S_L_LEG_LAX 15
#define PELV_S_R_LEG_UHZ 16
#define PELV_S_R_LEG_MHX 17
#define PELV_S_R_LEG_LHY 18
#define PELV_S_R_LEG_KNY 19
#define PELV_S_R_LEG_UAY 20
#define PELV_S_R_LEG_LAX 21
#define PELV_S_L_ARM_USY 22 // weird joint, axis = 0.000000 -0.500000 0.866025
#define PELV_S_L_ARM_SHX 23
#define PELV_S_L_ARM_ELY 24
#define PELV_S_L_ARM_ELX 25
#define PELV_S_L_ARM_UWY 26 // wrist
#define PELV_S_L_ARM_MWX 27
#define PELV_S_R_ARM_USY 28 // weird joint, axis = 0.000000 0.500000 0.866025
#define PELV_S_R_ARM_SHX 29
#define PELV_S_R_ARM_ELY 30
#define PELV_S_R_ARM_ELX 31
#define PELV_S_R_ARM_UWY 32 // wrist
#define PELV_S_R_ARM_MWX 33 

// sdfast Body index definitions (for sdpointf, etc.)
#define PELV_S_BODY_WORLD      -1
#define PELV_S_BODY_PELVIS     0
#define PELV_S_BODY_LOW_TORSO  1
#define PELV_S_BODY_MID_TORSO  2
#define PELV_S_BODY_UP_TORSO   3
#define PELV_S_BODY_HEAD       4
#define PELV_S_BODY_L_UGLUT    5
#define PELV_S_BODY_L_LGLUT    6
#define PELV_S_BODY_L_ULEG     7
#define PELV_S_BODY_L_LLEG     8
#define PELV_S_BODY_L_TALUS    9
#define PELV_S_BODY_L_FOOT     10
#define PELV_S_BODY_R_UGLUT    11
#define PELV_S_BODY_R_LGLUT    12
#define PELV_S_BODY_R_ULEG     13
#define PELV_S_BODY_R_LLEG     14
#define PELV_S_BODY_R_TALUS    15
#define PELV_S_BODY_R_FOOT     16
#define PELV_S_BODY_L_CALV     17
#define PELV_S_BODY_L_SCAP     18
#define PELV_S_BODY_L_UARM     19
#define PELV_S_BODY_L_LARM     20
#define PELV_S_BODY_L_FARM     21
#define PELV_S_BODY_L_HAND     22
#define PELV_S_BODY_R_CALV     23
#define PELV_S_BODY_R_SCAP     24
#define PELV_S_BODY_R_UARM     25
#define PELV_S_BODY_R_LARM     26
#define PELV_S_BODY_R_FARM     27
#define PELV_S_BODY_R_HAND     28
#define PELV_N_BODIES          29

// sdfast Joint index definitions (for sdhinget, etc.)
#define PELV_S_JOINT_ROOT_SIXDOF 0
#define PELV_S_JOINT_BACK_LBZ   1 // back
#define PELV_S_JOINT_BACK_MBY   2 
#define PELV_S_JOINT_BACK_UBX   3
#define PELV_S_JOINT_NECK_AY   4 // neck fe
#define PELV_S_JOINT_L_LEG_UHZ 5
#define PELV_S_JOINT_L_LEG_MHX 6
#define PELV_S_JOINT_L_LEG_LHY 7
#define PELV_S_JOINT_L_LEG_KNY 8
#define PELV_S_JOINT_L_LEG_UAY 9
#define PELV_S_JOINT_L_LEG_LAX 10
#define PELV_S_JOINT_R_LEG_UHZ 11
#define PELV_S_JOINT_R_LEG_MHX 12
#define PELV_S_JOINT_R_LEG_LHY 13
#define PELV_S_JOINT_R_LEG_KNY 14
#define PELV_S_JOINT_R_LEG_UAY 15
#define PELV_S_JOINT_R_LEG_LAX 16
#define PELV_S_JOINT_L_ARM_USY  17 // weird joint, axis = 0.000000 -0.500000 0.866025
#define PELV_S_JOINT_L_ARM_SHX 18
#define PELV_S_JOINT_L_ARM_ELY 19
#define PELV_S_JOINT_L_ARM_ELX 20
#define PELV_S_JOINT_L_ARM_UWY 21 // wrist
#define PELV_S_JOINT_L_ARM_MWX 22
#define PELV_S_JOINT_R_ARM_USY  23 // weird joint, axis = 0.000000 0.500000 0.866025
#define PELV_S_JOINT_R_ARM_SHX 24
#define PELV_S_JOINT_R_ARM_ELY 25
#define PELV_S_JOINT_R_ARM_ELX 26
#define PELV_S_JOINT_R_ARM_UWY 27 // wrist
#define PELV_S_JOINT_R_ARM_MWX 28  
//#define PELV_S_JOINT_L_FOOT_WELD    29
//#define PELV_S_JOINT_R_FOOT_WELD    30
//#define PELV_N_SDFAST_JOINTS        31
#define PELV_N_SDFAST_JOINTS        29

// end of SDfast defines
/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////

/* Robot joints (for trajectories and gains) */
#define PELV_A_BACK_LBZ  0 // back
#define PELV_A_BACK_MBY  1 
#define PELV_A_BACK_UBX  2
#define PELV_A_NECK_AY   3 // neck fe
#define PELV_A_L_LEG_UHZ 4
#define PELV_A_L_LEG_MHX 5
#define PELV_A_L_LEG_LHY 6
#define PELV_A_L_LEG_KNY 7
#define PELV_A_L_LEG_UAY 8
#define PELV_A_L_LEG_LAX 9
#define PELV_A_R_LEG_UHZ 10
#define PELV_A_R_LEG_MHX 11
#define PELV_A_R_LEG_LHY 12
#define PELV_A_R_LEG_KNY 13
#define PELV_A_R_LEG_UAY 14
#define PELV_A_R_LEG_LAX 15
#define PELV_A_L_ARM_USY 16 // weird joint, axis = 0.000000 -0.500000 0.866025
#define PELV_A_L_ARM_SHX 17
#define PELV_A_L_ARM_ELY 18
#define PELV_A_L_ARM_ELX 19
#define PELV_A_L_ARM_UWY 20 // wrist
#define PELV_A_L_ARM_MWX 21
#define PELV_A_R_ARM_USY 22 // weird joint, axis = 0.000000 0.500000 0.866025
#define PELV_A_R_ARM_SHX 23
#define PELV_A_R_ARM_ELY 24
#define PELV_A_R_ARM_ELX 25
#define PELV_A_R_ARM_UWY 26 // wrist
#define PELV_A_R_ARM_MWX 27   
#define PELV_N_JOINTS    28

// Points: useful for graphics, knowing where stuff is.
#define PELV_P_BACK_LBZ  0 // back
#define PELV_P_BACK_MBY  1 
#define PELV_P_BACK_UBX  2
#define PELV_P_NECK_AY   3 // neck fe
#define PELV_P_L_LEG_UHZ 4
#define PELV_P_L_LEG_MHX 5
#define PELV_P_L_LEG_LHY 6
#define PELV_P_L_LEG_KNY 7
#define PELV_P_L_LEG_UAY 8
#define PELV_P_L_LEG_LAX 9
#define PELV_P_R_LEG_UHZ 10
#define PELV_P_R_LEG_MHX 11
#define PELV_P_R_LEG_LHY 12
#define PELV_P_R_LEG_KNY 13
#define PELV_P_R_LEG_UAY 14
#define PELV_P_R_LEG_LAX 15
#define PELV_P_L_ARM_USY 16 // weird joint, axis = 0.000000 -0.500000 0.866025
#define PELV_P_L_ARM_SHX 17
#define PELV_P_L_ARM_ELY 18
#define PELV_P_L_ARM_ELX 19
#define PELV_P_L_ARM_UWY 20 // wrist
#define PELV_P_L_ARM_MWX 21
#define PELV_P_R_ARM_USY 22 // weird joint, axis = 0.000000 0.500000 0.866025
#define PELV_P_R_ARM_SHX 23
#define PELV_P_R_ARM_ELY 24
#define PELV_P_R_ARM_ELX 25
#define PELV_P_R_ARM_UWY 26 // wrist
#define PELV_P_R_ARM_MWX 27     
#define PELV_N_LOCATIONS 28

/************************************************************************/ 

#endif
