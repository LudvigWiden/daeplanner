std_msgs/Time stamp

# The name of the controller (implementation-wise).
# Beware, multiple instances of the same controller can be running.
# The ControlManagerDiagnostics message contains the name given them
# by the ControlManager.
string controller

# True if the controller is in the initial rampup phase (just after activation).
bool ramping_up

# true if the mass estimator is running
bool mass_estimator

# The estimated mass difference from the nominal mass.
float64 mass_difference

# Total estimated UAV mass.
float64 total_mass

# true if disturbance estimators are running
bool disturbance_estimator

# World-frame disturbances expressed in the world frame.
float64 disturbance_wx_w
float64 disturbance_wy_w

# Body-frame (fcu_untilted) disturbances expressed in the world frame.
float64 disturbance_bx_w
float64 disturbance_by_w

# Body-frame (fcu_untilted) disturbances expressed in the body frame (fcu_untilted).
float64 disturbance_bx_b
float64 disturbance_by_b

# The controller can enforce dynamics constraints over the trackers.
# This can be used when flying with a controller that is limited to
# some maximum speed and acceleration.
bool controller_enforcing_constraints
float64 horizontal_speed_constraint
float64 horizontal_acc_constraint
float64 vertical_asc_speed_constraint
float64 vertical_asc_acc_constraint
float64 vertical_desc_speed_constraint
float64 vertical_desc_acc_constraint
