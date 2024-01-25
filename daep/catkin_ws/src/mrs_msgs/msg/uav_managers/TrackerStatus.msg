bool active
bool callbacks_enabled

# is true whenever the tracker is doing something
# in other words, not idling
bool have_goal

# is true only then a trajectory is being tracked
bool tracking_trajectory

int32 trajectory_length
int32 trajectory_idx

# only when following trajectory
# the current reference in the trajectory UNINTERPOLATED
mrs_msgs/ReferenceStamped trajectory_reference
