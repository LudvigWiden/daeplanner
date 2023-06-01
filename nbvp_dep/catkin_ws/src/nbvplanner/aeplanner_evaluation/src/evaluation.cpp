#include <ros/ros.h>
#include <aeplanner_evaluation/Coverage.h>
#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include <vector>

void octomapCallback(const octomap_msgs::Octomap& msg);

class CoverageEvaluator{
  private:
    ros::NodeHandle nh_;
    ros::Subscriber octomap_sub_;
    ros::ServiceServer coverage_srv_;

    std::vector<double> min_vals;
    std::vector<double> max_vals;
    double volume_;
    double coverage_m3_, coverage_p_, free_, occupied_, unmapped_;
  public:
    CoverageEvaluator(ros::NodeHandle& nh) : nh_(nh),
                                             octomap_sub_(nh_.subscribe("octomap", 10, &CoverageEvaluator::octomapCallback, this)),
                                             coverage_srv_(nh.advertiseService("get_coverage", &CoverageEvaluator::coverageSrvCallback, this)){
      
      
      if (!nh.getParam("/firefly/nbvPlanner/bbx/volume", volume_)) {
        ROS_ERROR("No volume found...");
      }
      if (!nh.getParam("/firefly/nbvPlanner/boundary/min", min_vals)) {
        ROS_ERROR("No minimum boundary values found...");
      }
      if (!nh.getParam("/firefly/nbvPlanner/boundary/max", max_vals)) {
        ROS_ERROR("No maximum boundary values found...");
      }
    }

    void octomapCallback(const octomap_msgs::Octomap& msg);
    bool coverageSrvCallback(aeplanner_evaluation::Coverage::Request& request, aeplanner_evaluation::Coverage::Response& response);

};

int main(int argc, char** argv) {
  
  ros::init(argc, argv, "coverage_evaluation");
  ROS_INFO("coverage_evaluation initialized");
  ros::NodeHandle nh;

  CoverageEvaluator ce_(nh);

  ros::spin();
}

void CoverageEvaluator::octomapCallback(const octomap_msgs::Octomap& msg){
  octomap::AbstractOcTree* aot = octomap_msgs::msgToMap(msg);
  octomap::OcTree* octree = dynamic_cast<octomap::OcTree*>(aot);
  
  double occupied_space = 0.0;
  double free_space = 0.0;

  // Map limits
  octomap::point3d min(min_vals[0], min_vals[1], min_vals[2]);
  octomap::point3d max(max_vals[0], max_vals[1], max_vals[2]);

    for (octomap::OcTree::leaf_bbx_iterator it = octree->begin_leafs_bbx(min, max), end = octree->end_leafs_bbx(); it != end; ++it) {
      const octomap::point3d& pos = it.getCoordinate(); // get the position of the leaf node
      // Check if position is within the bounding box
      if (pos.x() > min.x() && pos.y() > min.y() && pos.z() > min.z() && pos.x() < max.x() && pos.y() < max.y() && pos.z() < max.z()) { 
        double side_length = it.getSize();
        if (octree->isNodeOccupied(*it)){ // occupied leaf node
          occupied_space += pow(side_length,3);
        } else { // free leaf node
          free_space += pow(side_length,3);
        }
      }
    }

    double mapped_space = occupied_space + free_space;
    double unmapped_space = volume_ - mapped_space;
    double coverage = 100.0 * mapped_space / volume_;

    // Set global variables
    coverage_m3_ = mapped_space;
    coverage_p_ = coverage;
    free_ = free_space;
    occupied_ = occupied_space;
    unmapped_ = unmapped_space;
}

bool CoverageEvaluator::coverageSrvCallback(aeplanner_evaluation::Coverage::Request& request, aeplanner_evaluation::Coverage::Response& response){
  response.coverage_m3 = coverage_m3_;
  response.coverage_p = coverage_p_;
  response.free     = free_;
  response.occupied = occupied_;
  response.unmapped = unmapped_;
  return true;
}
