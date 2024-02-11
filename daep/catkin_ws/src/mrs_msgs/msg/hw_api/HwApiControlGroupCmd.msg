time stamp

# 4DOF control vector, in the Front-Left-Up body frame
#
# The control group signals are first passed through a mixer matrix,
# then desaturated and applied to the motor outputs [0, 1].
float32 roll # [-1, 1]
float32 pitch # [-1, 1]
float32 yaw # [-1, 1]
float32 throttle # [0, 1]
