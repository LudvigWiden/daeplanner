Header header

int64 input_id

bool use_heading
bool fly_now
bool stop_at_waypoints
bool loop

# how much time [s] is given to the solver (fallback sampling is triggered after that)
# used when > 0, value from the config is used otherwise
float64 max_execution_time

# how much [s] can the resulting trajectory deviate from the path
# used when > 0, value from the config is used otherwise
float64 max_deviation_from_path

# when true, the by-default-ON prepanding of the current state is disabled
bool dont_prepend_current_state

bool override_constraints
float64 override_max_velocity_horizontal
float64 override_max_acceleration_horizontal
float64 override_max_jerk_horizontal
float64 override_max_velocity_vertical
float64 override_max_acceleration_vertical
float64 override_max_jerk_vertical

bool relax_heading

Reference[] points
