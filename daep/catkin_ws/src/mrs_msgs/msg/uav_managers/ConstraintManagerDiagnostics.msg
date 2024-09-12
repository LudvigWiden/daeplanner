# The diagnostics of the ConstraintManager.

time stamp

# The list of all loaded constraints.
string[] loaded

# The list of constraints which are available for the currently active state estimator.
string[] available

# The name of the current constraints.
string current_name

# The particular values of the current constraints.
# Beware, those might be overloaded by the current controller.
# Subscribe to "control_manager/current_constraints" for the
# true values.
mrs_msgs/DynamicsConstraints current_values
