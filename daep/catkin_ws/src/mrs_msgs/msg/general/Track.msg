# This message represents an object being tracked in a three-dimensional
# free space by a multi-target tracking algorithm.
# Each tracked objects has a unique and persistent track ID,
# a position, orientation and velocity estimate with uncertainties represented
# by covariance matrices.

# An ID of this track that is unique and persistent for the tracked object.
uint64 id

# The rest of this message contains the state estimate of the track,
# including its position, orientation, and velocity and their respective
# uncertainty covariance matrices. These are all expressed in a coordinate
# frame, that is given in the header of the meta-message, containing this message.

# Three-dimensional position of the track.
geometry_msgs/Point position

# Row-major representation of the 3x3 matrix of position covariance.
# In order, the parameters are:
# (x, y, z)
float64[9] position_covariance

# Three-dimensional orientation of the track.
geometry_msgs/Quaternion orientation

# Row-major representation of the 3x3 matrix of orientation covariance.
# The orientation parameters use a fixed-axis representation.
# In order, the parameters are:
# (rotation about X axis, rotation about Y axis, rotation about Z axis)
float64[9] orientation_covariance

# Velocity of the track in free space.
geometry_msgs/Vector3 velocity

# Row-major representation of the 3x3 matrix of velocity covariance.
# In order, the parameters are:
# (dx/dt, dy/dt, dz/dt)
float64[9] velocity_covariance
