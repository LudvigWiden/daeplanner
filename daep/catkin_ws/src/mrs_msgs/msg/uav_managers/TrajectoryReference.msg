# The Trajectory reference that can be supplied to mrs_uav_manager::ControlManager.

Header header

# uniquely identifies the user input
int64 input_id

# True if the heading part should be tracked, otherwise the tracker should maintain the current heading.
bool use_heading

# True if the trajectory should start being tracked immediately after receiving it.
# Beware the header time stamp matters in this case.
bool fly_now

# True if the trajectory should be looped.
bool loop

# The time difference between the trajectory samples.
# It falls back to 0.2 s if left unset.
float64 dt

# The list of points in the trajectory, spaced by the dt.
Reference[] points 
