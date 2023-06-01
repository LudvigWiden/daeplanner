
#ifndef _RRT_H_
#define _RRT_H_

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>

#include <eigen3/Eigen/Dense>

#include <rrtplanner/rrtAction.h>
#include <actionlib/server/simple_action_server.h>

#include <kdtree/kdtree.h>
#include <nav_msgs/Path.h>

// DAEP
#include <gazebo_msgs/ModelStates.h>
#include <vector>
#include <utility>
#include <visualization_msgs/Marker.h>
#include <rrtplanner/kalman.h>
#include <tf/transform_datatypes.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <algorithm>
#include <iostream>
#include <tuple>
#include <pigain/Query.h>

#include <rrtplanner/SafePath.h>


namespace aeplanner_ns
{

struct RrtNode
{
  Eigen::Vector3d pos;
  RrtNode *parent{NULL};
  std::vector<RrtNode *> children;

  ~RrtNode()
  {
    for (typename std::vector<RrtNode *>::iterator node_it = children.begin();
         node_it != children.end(); ++node_it)
    {
      delete (*node_it);
      (*node_it) = NULL;
    }
  }

  double cost()
  {
    if (parent)
      return (pos - parent->pos).norm() + parent->cost();
    return 0;
  }

  double time_cost(double drone_linear_velocity, double drone_angular_velocity)
  {
    if (this->parent)
      return this->time_to_reach(this->parent->pos, drone_linear_velocity, drone_angular_velocity) + this->parent->time_cost(drone_linear_velocity, drone_angular_velocity);
    else
      return 0;
  }

  double time_to_reach(const Eigen::Vector3d& pos, double drone_linear_velocity, double drone_angular_velocity) 
  {
    //Current node
    double current_x = this->pos[0];
    double current_y = this->pos[1];
    double current_z = this->pos[2];

    //New node to reach
    double target_x = pos[0];
    double target_y = pos[1];
    double target_z = pos[2];
    
    //Angle to rotate from current direction to new node direction
    double angle = std::acos(this->pos.dot(pos) / (this->pos.norm() * pos.norm()));

    // Calculate time to rotate to target yaw angle
    double rotation_time = fabs(angle) / drone_angular_velocity;

    // Calculate Euclidean distance in x-y plane
    Eigen::Vector3d p3(this->pos[0], this->pos[1], this->pos[2]);
    Eigen::Vector3d q3(pos[0], pos[1], pos[2]);
    double euclidean_distance = (p3 - q3).norm();

    // Calculate time to move linearly to target position
    double linear_time = euclidean_distance / drone_linear_velocity;

    // Calculate total time to reach target state
    double total_time = rotation_time + linear_time;
    return total_time;
  }

};

class Rrt
{
  public:
    Rrt(const ros::NodeHandle &nh);
    void octomapCallback(const octomap_msgs::Octomap &msg);

    void execute(const rrtplanner::rrtGoalConstPtr &goal);
    void visualizeGoals(std::vector<geometry_msgs::Pose> goals);
    void visualizeNode(geometry_msgs::Point pos, int id = 0);
    void visualizePose(geometry_msgs::Pose pose, int id = 0);
    void visualizeEdge(RrtNode *node, int id = 0);
    void visualizePath(RrtNode *node);

    Eigen::Vector3d sample();
    RrtNode *chooseParent(kdtree *kd_tree, Eigen::Vector3d, double l);
    void rewire(kdtree *kd_tree, RrtNode *new_node, double l, double r, double r_os);
    Eigen::Vector3d getNewPosNormalized(Eigen::Vector3d sampled, Eigen::Vector3d parent, double l);
    bool collisionLine(Eigen::Vector3d p1, Eigen::Vector3d p2, double r);
    RrtNode *addNodeToTree(kdtree *kd_tree, RrtNode *parent, Eigen::Vector3d new_pos);
    RrtNode *getGoal(kdtree *goal_tree, RrtNode *new_node, double l, double r, double r_os);
    std::pair<nav_msgs::Path, std::vector<double>> getBestPath(std::vector<RrtNode*> goals);
    std::vector<geometry_msgs::Pose> checkIfGoalReached(kdtree *goal_tree, RrtNode *new_node, double l, double r, double r_os);

  private:
    ros::NodeHandle nh_;
    std::shared_ptr<octomap::OcTree> ot_;

    ros::Subscriber octomap_sub_;
    actionlib::SimpleActionServer<rrtplanner::rrtAction> as_;

    std::string frame_id_;

    ros::Publisher path_pub_;
    double min_nodes_;
    double bounding_radius_;
    double bounding_overshoot_;
    double extension_range_;
    std::vector<double> boundary_min_;
    std::vector<double> boundary_max_;
    int KFiterations_;
    double dt_;

    double drone_linear_velocity;
    double drone_angular_velocity;

    //SAFE PATH with DAEP
    std::map<std::string, std::pair<geometry_msgs::Pose, geometry_msgs::Twist>> dynamic_objects;
    ros::Subscriber human_sub_;
    void updateHumanPositions(const gazebo_msgs::ModelStates& model_states);
    std::vector<std::pair<std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>, std::vector<Eigen::MatrixXd>>> KFpredictTrajectories();
    bool dynamic_mode_;
    bool safePathSrvCallback(rrtplanner::SafePath::Request& request, rrtplanner::SafePath::Response& response);
    ros::ServiceServer safe_path_srv;
    bool isCollision(const geometry_msgs::PoseStamped& posestamped_parent, const geometry_msgs::PoseStamped& posestamped, double time_of_arrival,
                          std::vector<std::pair<std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>, std::vector<Eigen::MatrixXd>>> predicted_data);
    bool checkCollision(double t, 
                        Eigen::Vector3d point, 
                        std::vector<std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>> trajectories, 
                        std::vector<std::vector<std::tuple<double, double, Eigen::MatrixXd>>> covarianceEllipses,
                        Eigen::Vector3d parent);
                          
    bool lineIntersectsCircle(const Eigen::Vector3d p1, const Eigen::Vector3d p2,
                            const Eigen::Vector3d center, double radius);

    int getCovarianceIndex(double max_time_step, double time_step, double t);
    std::vector<std::tuple<double, double, Eigen::MatrixXd>> createCovarianceEllipse(const std::vector<Eigen::MatrixXd>& cov_matrices);

};
  

float CylTest_CapsFirst(const octomap::point3d &pt1,
                        const octomap::point3d &pt2,
                        float lsq, float rsq, const octomap::point3d &pt);
} // namespace aeplanner_ns

#endif
