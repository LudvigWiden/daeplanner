# This message represents an Image and corresponding camera label
# Can be suitable for any multi camera setup as UVDar or stereo pair
# to synchronise all images on a ROS level (check all timestamps and
# collect them in one array with one timestamp)

Header header
mrs_msgs/ImageLabeled[] imgs_labeled
