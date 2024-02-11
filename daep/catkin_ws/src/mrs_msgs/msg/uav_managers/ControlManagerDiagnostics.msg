# This reports on the state of the ControlManager and the currently active tracker and controller.

time stamp
string uav_name

# If the motors are enabled, in other words, if the output of the ControlManager can pass out.
bool output_enabled

# True if the UAV is flying and no special case occurred (emergency land, failsafe, etc.).
bool flying_normally

# True if under the "RC Joystick" mode
bool rc_mode

# The recommended (but not lower than) desired rate of incoming UAV State
float64 desired_uav_state_rate

# The name of the currently active tracker, as defined in the trackers.yaml in the "mrs_uav_manager" package.
string active_tracker

# The status of the currently active tracker.
mrs_msgs/TrackerStatus tracker_status

# The name of the currently active controller, as defined in the controllers.yaml in the "mrs_uav_manager" package.
string active_controller

# The status of the currently active controller.
mrs_msgs/ControllerStatus controller_status

# list of controllers and trackers that are loaded and can be switched to
string[] available_controllers
string[] available_trackers

# flags reporting on whether the trackers and controller should be "human switchable", meaning,
# whether people should be "just" able to switch to them, e.g., using a GUI or TUI interface
bool[] human_switchable_controllers
bool[] human_switchable_trackers
