#ifndef __DRC_HATNS_DEFINES
#define __DRC_HATNS_DEFINES

#define HATNS_N_SDFAST_Q     19
#define HATNS_N_SDFAST_U     18
#define HATNS_N_SDFAST_STATE (HATNS_N_SDFAST_U+HATNS_N_SDFAST_Q)

/* Indices for sdfast state */
#define HATNS_S_X     0
#define HATNS_S_Y     1
#define HATNS_S_Z     2
#define HATNS_S_Q0    3
#define HATNS_S_Q1    4
#define HATNS_S_Q2    5
#define HATNS_S_Q3    18

#define HATNS_S_L_LEG_UHZ   6
#define HATNS_S_L_LEG_MHX   7
#define HATNS_S_L_LEG_LHY   8
#define HATNS_S_L_LEG_KNY   9
#define HATNS_S_L_LEG_UAY   10
#define HATNS_S_L_LEG_LAX   11
#define HATNS_S_R_LEG_UHZ   12
#define HATNS_S_R_LEG_MHX   13
#define HATNS_S_R_LEG_LHY   14
#define HATNS_S_R_LEG_KNY   15
#define HATNS_S_R_LEG_UAY   16
#define HATNS_S_R_LEG_LAX   17

// sdfast Body index definitions (for sdpointf, etc.)
#define HATNS_S_BODY_WORLD      -1
#define HATNS_S_BODY_PELVIS     0
#define HATNS_S_BODY_LOW_TORSO  1
#define HATNS_S_BODY_MID_TORSO  2
#define HATNS_S_BODY_HAT        3
#define HATNS_S_BODY_L_UGLUT    4
#define HATNS_S_BODY_L_LGLUT    5
#define HATNS_S_BODY_L_ULEG     6
#define HATNS_S_BODY_L_LLEG     7
#define HATNS_S_BODY_L_TALUS    8
#define HATNS_S_BODY_L_FOOT     9
#define HATNS_S_BODY_R_UGLUT    10
#define HATNS_S_BODY_R_LGLUT    11
#define HATNS_S_BODY_R_ULEG     12
#define HATNS_S_BODY_R_LLEG     13
#define HATNS_S_BODY_R_TALUS    14
#define HATNS_S_BODY_R_FOOT     15
#define HATNS_N_BODIES          16

// sdfast Joint index definitions (for sdhinget, etc.)
#define HATNS_S_JOINT_ROOT_SIXDOF 0
#define HATNS_S_JOINT_BACK_LBZ    1 // back
#define HATNS_S_JOINT_BACK_MBY    2 
#define HATNS_S_JOINT_BACK_UBX    3
#define HATNS_S_JOINT_L_LEG_UHZ   4
#define HATNS_S_JOINT_L_LEG_MHX   5
#define HATNS_S_JOINT_L_LEG_LHY   6
#define HATNS_S_JOINT_L_LEG_KNY   7
#define HATNS_S_JOINT_L_LEG_UAY   8
#define HATNS_S_JOINT_L_LEG_LAX   9
#define HATNS_S_JOINT_R_LEG_UHZ  10
#define HATNS_S_JOINT_R_LEG_MHX  11
#define HATNS_S_JOINT_R_LEG_LHY  12
#define HATNS_S_JOINT_R_LEG_KNY  13
#define HATNS_S_JOINT_R_LEG_UAY  14
#define HATNS_S_JOINT_R_LEG_LAX  15
#define HATNS_N_SDFAST_JOINTS    16

// end of SDfast defines
/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////

/* Robot joints (for trajectories and gains) */
#define HATNS_A_L_LEG_UHZ 0
#define HATNS_A_L_LEG_MHX 1
#define HATNS_A_L_LEG_LHY 2
#define HATNS_A_L_LEG_KNY 3
#define HATNS_A_L_LEG_UAY 4
#define HATNS_A_L_LEG_LAX 5
#define HATNS_A_R_LEG_UHZ 6
#define HATNS_A_R_LEG_MHX 7
#define HATNS_A_R_LEG_LHY 8
#define HATNS_A_R_LEG_KNY 9
#define HATNS_A_R_LEG_UAY 10
#define HATNS_A_R_LEG_LAX 11
#define HATNS_N_JOINTS    12

/************************************************************************/ 

#endif 
