# Range: [0, 1]
float64 joy_axis_deadzone
  
# Range: [0, 1]
float64 max_speed_cmd
float64 max_heave_cmd
float64 max_roll_cmd
float64 max_pitch_cmd
float64 max_yaw_cmd
  
# Recommended range: [0, 180] (degrees)
float64 max_roll_pos
float64 max_pitch_pos
float64 max_yaw_pos

# Recommended range: [-2, 1] (m)
float64 min_depth

# Recommended range: [0, 100] (m)
float64 max_depth

# Use fixed initial depth (m) upon entering depth-related modes [if < 0, then initial depth is current robot depth]
float64 default_fixed_depth
  
# Recommended range: [0, 180] (deg/s)
float64 max_roll_vel
float64 max_pitch_vel
float64 max_yaw_vel
  
# Recommended range: [0, 1] (m/s)
float64 max_depth_vel
