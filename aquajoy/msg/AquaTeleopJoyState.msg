int8 ctrl_mode
int8 CTRL_MODE_RAW = 0
int8 CTRL_MODE_GLOBAL_RP_POS = 1
int8 CTRL_MODE_GLOBAL_RP_POS_D = 2
int8 CTRL_MODE_GLOBAL_RP_VEL_D = 3
int8 CTRL_MODE_FLATSWIM_D = 4

bool modifier

aquacore/Command raw_cmd

# NOTE: all angle units are in degrees

float64 roll_pos
float64 pitch_pos
float64 yaw_pos

float64 roll_vel
float64 pitch_vel
float64 yaw_vel

float64 depth_pos

float64 depth_vel
