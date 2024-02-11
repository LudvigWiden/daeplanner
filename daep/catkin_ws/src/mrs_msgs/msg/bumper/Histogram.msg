# This is a debug message for the mrs_bumper package.
# It contains a histogram of depth values from a depthmap
# image, if the depth sensor is used for obstacle detection.

# Time at which the data in this message was generated, coordinate frame ID.
std_msgs/Header header

# Unit of distance measurement in the histogram (e.g. "mm" for millimeters").
string    unit

# Structure of the histogram.
float64   bin_size
float64   bin_min
float64   bin_max

# The mark at which an obstacle was detected (if equal to the number of bins,
# no obstacle was detected).
uint32    bin_mark

# Bin values of the histogram.
float32[] bins
