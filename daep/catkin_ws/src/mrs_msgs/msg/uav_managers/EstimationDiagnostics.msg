std_msgs/Header header

# frame of velocities and accelerations
string child_frame_id

# current estimated state (corresponds to uav_state msg)
geometry_msgs/Pose pose
geometry_msgs/Twist velocity
geometry_msgs/Accel acceleration

# agl height (above ground level)
float64 agl_height

# max allowed flight height (defined by safety area and available sensors)
float64 max_flight_z

# current state machine state
string sm_state

# current state estimator used for control
string current_state_estimator

# current subestimators used for control
string estimator_horizontal
string estimator_vertical
string estimator_heading

# current agl height estimator
string estimator_agl_height

# rate of estimation prediction and uav_state publishing
uint32 estimation_rate

# number of performed estimators switches
uint32 estimator_iteration

# estimators that are running
string[] running_state_estimators

# estimators that are healthy and ready to be switch into
string[] switchable_state_estimators

# loaded platform config file
string platform_config

# loaded custom config file
string custom_config
