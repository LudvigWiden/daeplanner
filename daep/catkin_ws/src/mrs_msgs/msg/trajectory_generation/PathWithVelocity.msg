Header header

int64 input_id

bool use_heading
bool fly_now
bool stop_at_waypoints
bool loop

# if this is enabled, the slower velocity between the points and these overrides will be chosen
bool override_constraints
float64 override_max_velocity_horizontal
float64 override_max_acceleration_horizontal
float64 override_max_velocity_vertical
float64 override_max_acceleration_vertical

bool relax_heading

ReferenceWithVelocity[] points
