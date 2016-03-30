# All of the following are pseudo-rates, meaning that they are correlated to the distance / time in a
# non-linear way; see Giguere et al. IROS 2006 "Characterizing..." for similar pseudo-rate mapping plots
# on Aqua 1.0. Do keep in mind that these plots do not apply for Aqua 2.0+, which do not have proper mappings yet.

# All saturations are performed by RoboDevel

float32 speed # unitless pseudo-rate, [0, 1]
float32 yaw   # unitless pseudo-rate, [-1, 1] # +1: yaw right
float32 pitch # unitless pseudo-rate, [-1, 1] # +1: pitch forwards down
float32 roll  # unitless pseudo-rate, [-1, 1] # +1: roll right
float32 heave # unitless pseudo-rate, [-1, 1] # +1: heave up

