# This message represents the output from the mrs_bumper package.
#
# The *sectors* variable contains information about obstacles in the horizontal,
# bottom and up sectors. Horizontal sectors take up the first part of the array.
# Bottom and up sectors are the second and first elements from the end of the array:
# sectors[0, 1, ..., n_horizontal_sectors-1] -> horizontal sectors
# sectors[n_horizontal_sectors]              -> bottom sector
# sectors[n_horizontal_sectors+1]            -> top sector
#
# The horizontal sectors are indexed counterclockwise, starting from the front facing
# sector, relative to the MAV orientation (see picture below). The last two
# elements of the *sectors* variable are the above and below sectors, respectively.
#
#         TOP VIEW
#         --------
#
#        \         /
#         \   0   /   (forward)
#          \     /        x 
#                         ^
#            O O          |
#        1    X    3   <--* z
#            O O       y
#                
#          /     \
#         /   2   \
#        /         \

# Time at which the data in this message was generated, coordinate frame ID.
std_msgs/Header header

# Number of horizontal bumper sectors (total number of sectors -2).
uint32    n_horizontal_sectors

# Vertical FOV of the individual horizontal sectors.
float64   sectors_vertical_fov

# Distance to closest detected obstacle in each sector.
float64[] sectors

# Sensor ID of the sensor, which was used for obstacle detection in each sector.
int8[]    sector_sensors

## Obstacle distance special values.
float64 OBSTACLE_NOT_DETECTED=-1
float64 OBSTACLE_NO_DATA=-2

## Sensor types enum.
int8 SENSOR_NONE=-1
int8 SENSOR_DEPTH=0
int8 SENSOR_LIDAR1D=1
int8 SENSOR_LIDAR2D=2
int8 SENSOR_LIDAR3D=3
