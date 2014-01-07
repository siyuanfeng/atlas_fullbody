#ifndef __DRC_HAT_DEFINES
#define __DRC_HAT_DEFINES

#define HAT_N_SDFAST_Q     22
#define HAT_N_SDFAST_U     21
#define HAT_N_SDFAST_STATE (HAT_N_SDFAST_U+HAT_N_SDFAST_Q)

/* Indices for sdfast state */
#define HAT_S_X     0
#define HAT_S_Y     1
#define HAT_S_Z     2
#define HAT_S_Q0    3
#define HAT_S_Q1    4
#define HAT_S_Q2    5
#define HAT_S_Q3    21

#define HAT_S_BACK_LBZ   6 // back
#define HAT_S_BACK_MBY   7 
#define HAT_S_BACK_UBX   8
#define HAT_S_L_LEG_UHZ  9
#define HAT_S_L_LEG_MHX 10
#define HAT_S_L_LEG_LHY 11
#define HAT_S_L_LEG_KNY 12
#define HAT_S_L_LEG_UAY 13
#define HAT_S_L_LEG_LAX 14
#define HAT_S_R_LEG_UHZ 15
#define HAT_S_R_LEG_MHX 16
#define HAT_S_R_LEG_LHY 17
#define HAT_S_R_LEG_KNY 18
#define HAT_S_R_LEG_UAY 19
#define HAT_S_R_LEG_LAX 20

// sdfast Body index definitions (for sdpointf, etc.)
#define HAT_S_BODY_WORLD      -1
#define HAT_S_BODY_PELVIS     0
#define HAT_S_BODY_LOW_TORSO  1
#define HAT_S_BODY_MID_TORSO  2
#define HAT_S_BODY_HAT        3
#define HAT_S_BODY_L_UGLUT    4
#define HAT_S_BODY_L_LGLUT    5
#define HAT_S_BODY_L_ULEG     6
#define HAT_S_BODY_L_LLEG     7
#define HAT_S_BODY_L_TALUS    8
#define HAT_S_BODY_L_FOOT     9
#define HAT_S_BODY_R_UGLUT    10
#define HAT_S_BODY_R_LGLUT    11
#define HAT_S_BODY_R_ULEG     12
#define HAT_S_BODY_R_LLEG     13
#define HAT_S_BODY_R_TALUS    14
#define HAT_S_BODY_R_FOOT     15
#define HAT_N_BODIES          16

// sdfast Joint index definitions (for sdhinget, etc.)
#define HAT_S_JOINT_ROOT_SIXDOF 0
#define HAT_S_JOINT_BACK_LBZ    1 // back
#define HAT_S_JOINT_BACK_MBY    2 
#define HAT_S_JOINT_BACK_UBX    3
#define HAT_S_JOINT_L_LEG_UHZ   4
#define HAT_S_JOINT_L_LEG_MHX   5
#define HAT_S_JOINT_L_LEG_LHY   6
#define HAT_S_JOINT_L_LEG_KNY   7
#define HAT_S_JOINT_L_LEG_UAY   8
#define HAT_S_JOINT_L_LEG_LAX   9
#define HAT_S_JOINT_R_LEG_UHZ  10
#define HAT_S_JOINT_R_LEG_MHX  11
#define HAT_S_JOINT_R_LEG_LHY  12
#define HAT_S_JOINT_R_LEG_KNY  13
#define HAT_S_JOINT_R_LEG_UAY  14
#define HAT_S_JOINT_R_LEG_LAX  15
#define HAT_N_SDFAST_JOINTS    16

// end of SDfast defines
/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////

/* Robot joints (for trajectories and gains) */
#define HAT_A_BACK_LBZ  0 // back
#define HAT_A_BACK_MBY  1 
#define HAT_A_BACK_UBX  2
#define HAT_A_L_LEG_UHZ 3
#define HAT_A_L_LEG_MHX 4
#define HAT_A_L_LEG_LHY 5
#define HAT_A_L_LEG_KNY 6
#define HAT_A_L_LEG_UAY 7
#define HAT_A_L_LEG_LAX 8
#define HAT_A_R_LEG_UHZ 9
#define HAT_A_R_LEG_MHX 10
#define HAT_A_R_LEG_LHY 11
#define HAT_A_R_LEG_KNY 12
#define HAT_A_R_LEG_UAY 13
#define HAT_A_R_LEG_LAX 14
#define HAT_N_JOINTS    15

// Points: useful for graphics, knowing where stuff is.
#define HAT_P_BACK_LBZ  0 // back
#define HAT_P_BACK_MBY  1 
#define HAT_P_BACK_UBX  2
#define HAT_P_L_LEG_UHZ 3
#define HAT_P_L_LEG_MHX 4
#define HAT_P_L_LEG_LHY 5
#define HAT_P_L_LEG_KNY 6
#define HAT_P_L_LEG_UAY 7
#define HAT_P_L_LEG_LAX 8
#define HAT_P_R_LEG_UHZ 9
#define HAT_P_R_LEG_MHX 10
#define HAT_P_R_LEG_LHY 11
#define HAT_P_R_LEG_KNY 12
#define HAT_P_R_LEG_UAY 13
#define HAT_P_R_LEG_LAX 14
#define HAT_N_LOCATIONS 15

/************************************************************************/ 

#endif
