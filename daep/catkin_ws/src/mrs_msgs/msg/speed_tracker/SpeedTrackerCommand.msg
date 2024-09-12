# contains the frame_id = your reference frame of choice
std_msgs/Header header

# desired velocity [m/s], optional
geometry_msgs/Vector3 velocity

# desired acceleration [m/s^2], optional
geometry_msgs/Vector3 acceleration

# desired force [N], optional
geometry_msgs/Vector3 force

# z coordinate
float64 z

# desired heading and its rate, optional
# heading is "the azimuth of the body-x axis of the drone, a.k.a, the aircraft's front"
float64 heading
float64 heading_rate

# set to "true" if you want to control the state
bool use_velocity
bool use_acceleration
bool use_force
bool use_z
bool use_heading
bool use_heading_rate
