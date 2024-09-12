# This represents the output of the currently active Tracker (mrs_uav_manager::Tracker).
# This message is returned from a tracker by means of the update() function, called by the mrs_uav_manager::ControlManager.

std_msgs/Header header

# The desired position.
geometry_msgs/Point position

# The desired velocity.
geometry_msgs/Vector3 velocity

# The desired acceleration.
geometry_msgs/Vector3 acceleration

# The desired jerk.
geometry_msgs/Vector3 jerk

# The desired snap.
geometry_msgs/Vector3 snap

# all-state future prediction
mrs_msgs/MpcPredictionFullState full_state_prediction

# The desired orientation and attitude rate.
# It is mutually exclusive to heading+heading_rate.
geometry_msgs/Quaternion orientation
geometry_msgs/Point attitude_rate

# when used, it will override the throttle output of the controller
float64 throttle

# The desired heading and heading rate.
# It is mutually exclusive to orientation+attitude_rate.
float64 heading
float64 heading_rate
float64 heading_acceleration
float64 heading_jerk

# Whether the controller should mute its position feedback.
# Useful, e.g., during takeoff or in situations where sudden control
# error is expected but not immediately required to be fixed.
bool disable_position_gains

# Whether the controller should disable its antiwindups
bool disable_antiwindups

# Flags defining whether the XY or Z position reference was filled in.
# If not, the controllers should pay no attention to it and to the corresponding control error.
uint8 use_position_horizontal
uint8 use_position_vertical

# Flags defining whether the XY or Z velocity reference was filled in.
# If not, the controllers should pay no attention to it and to the corresponding control error.
uint8 use_velocity_horizontal
uint8 use_velocity_vertical

# Flags defining which references were filled in.
# If not, the controllers should pay no attention to them and to the corresponding control errors.
uint8 use_acceleration
uint8 use_jerk
uint8 use_snap
uint8 use_attitude_rate
uint8 use_heading
uint8 use_heading_rate
uint8 use_heading_acceleration
uint8 use_heading_jerk
uint8 use_orientation
uint8 use_throttle
uint8 use_full_state_prediction
