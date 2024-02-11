Header header

string uav_name
string nato_name
string uav_type
string uav_mass

float32 control_manager_diag_hz
int16 control_manager_diag_color
string[] controllers
string[] gains
string[] trackers
string[] constraints
string fly_state
bool null_tracker
uint32 secs_flown

float32 odom_hz
int16 odom_color
float32 odom_x
float32 odom_y
float32 odom_z
float32 odom_hdg
string odom_frame
string[] odom_estimators_hori
string[] odom_estimators_vert
string[] odom_estimators_hdg

float32 cmd_x
float32 cmd_y
float32 cmd_z
float32 cmd_hdg

float32 cpu_load
float32 cpu_load_total
float32 cpu_ghz
float32 cpu_temperature
float32 free_ram
float32 total_ram
int32 free_hdd

float32 hw_api_hz
int16 hw_api_color
float32 hw_api_battery_hz
float32 hw_api_state_hz
float32 hw_api_cmd_hz
string hw_api_mode
bool hw_api_armed
bool hw_api_gnss_ok
float32 hw_api_gnss_qual
float32 mag_norm_hz
float32 mag_norm
float32 battery_volt
float32 battery_curr
float32 battery_wh_drained
float32 thrust
float32 mass_estimate
float32 mass_set

CustomTopic[] custom_topics
string[] custom_string_outputs
string[] custom_services

NodeCpuLoad node_cpu_loads

bool flying_normally
bool have_goal
bool callbacks_enabled
bool collision_avoidance_enabled
bool avoiding_collision
bool automatic_start_can_takeoff

uint16 num_other_uavs
