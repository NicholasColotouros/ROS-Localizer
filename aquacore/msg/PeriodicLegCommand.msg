# This message represents the leg command parameterization that was decided by consensus at 
# MRL in July, 2014. The intention is that ROS-coded gaits will publish messages of this
# type, and that the aqua robot as well as its simulator will implement leg motions 
# that follow the requested sinusoidal trajectories.
#
# Author: David Meger
# Date: July 29, 2014 

Header header

#
# The ordering of the 6 legs in all following arrays is:
# 0: front-left
# 1: mid-left
# 2: hind-left
# 3: front-right
# 4: mid-right
# 5: hind-right
#
# The orientation of the leg angles are:
# 0: pointing backwards
# pi: pointing downwards
# 2*pi: pointing forwards
#
# Computation of current leg angle:
# leg_angle = amplitude * cos(2*pi * frequency * dt + phase_offset) + leg_offset
#
# Midoff positioning and velocity modes:
#
# The midoff angle of each leg can be commanded in either position mode or in
# velocity mode. They are implemented as follows:
#
# if leg_velocities[i] != 0:
#   actual leg offset = last internally-stored leg offset + leg_velocities[i]*dt
# else:
#   actual leg offset = leg_offsets[i]

float32[6] amplitudes     # units: rad
float32[6] frequencies    # units: hz
float32[6] phase_offsets  # units: rad
float32[6] leg_offsets    # units: rad
float32[6] leg_velocities # units: rad/s
