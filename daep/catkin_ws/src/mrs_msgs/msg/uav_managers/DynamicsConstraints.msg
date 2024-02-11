# UAV dynamics constraints

# speed: m/s
# acceleration: m/s^2
# jerk: m/s^3
# snap: m/s^4

# translational dynamics

float64 horizontal_speed
float64 horizontal_acceleration
float64 horizontal_jerk
float64 horizontal_snap

float64 vertical_ascending_speed
float64 vertical_ascending_acceleration
float64 vertical_ascending_jerk
float64 vertical_ascending_snap

float64 vertical_descending_speed
float64 vertical_descending_acceleration
float64 vertical_descending_jerk
float64 vertical_descending_snap

# heading

float64 heading_speed
float64 heading_acceleration
float64 heading_jerk
float64 heading_snap

# angular rates: rad/s

float64 roll_rate
float64 pitch_rate
float64 yaw_rate

# maximum tilt, rad
# max angle between body-3 and world-3 axes

float64 tilt
