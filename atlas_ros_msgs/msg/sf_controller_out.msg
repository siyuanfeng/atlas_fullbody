Header header

bool isPause
int32 smIndex

# state estimator stuff
#float64[28] joints
#float64[3]  root
#float64[4]  quat

float64[3] model_com

# ikcmd 
float64[3] ikcmd_com
float64[3] ik_com

# ik res
float64[28] ik_joints
float64[3]  ik_root
float64[4]  ik_pelv_quat

# id  
float64[4] id_foot_b_cop
float64[12] id_b_grf


# sf task stuff
int8 scriptMode
int8 mode
int8 numStepsLeftInScirpt

int8 T2_SCRIPT_MANUAL     = 90
int8 T2_SCRIPT_RAMP       = 91
int8 T2_SCRIPT_ZIGZAG     = 92
int8 T2_SCRIPT_FLATTOP    = 93
int8 T2_SCRIPT_TILTED     = 94
int8 T2_SCRIPT_IN_PLACE   = 95
int8 T2_SCRIPT_BIG_STRIDE = 96

int8 T2_MODE_HALT         = -1
int8 T2_MODE_FLAT         = 0
int8 T2_MODE_UPBLOCK      = 1
int8 T2_MODE_DOWNBLOCK    = 2
int8 T2_MODE_UPSLOPE      = 3
int8 T2_MODE_DOWNSLOPE    = 4

