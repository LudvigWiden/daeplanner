#include <rrtplanner/rrt.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rrt");
  ROS_INFO("Global RRT-planner initialized");
  ros::NodeHandle nh;

  aeplanner_ns::Rrt rrt(nh);

  ros::spin();
  return 0;
}
