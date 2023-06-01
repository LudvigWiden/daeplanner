#ifndef _AEPVIZ_H_
#define _AEPVIZ_H_

#include <string>
#include <tuple>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <eigen3/Eigen/Dense>
#include <tf/transform_datatypes.h>
#include <aeplanner/param.h>



#include <aeplanner/data_structures.h>

namespace aeplanner {
  visualization_msgs::MarkerArray createRRTMarkerArray(RRTNode * root, double lambda);
  void recurse(RRTNode * node, visualization_msgs::MarkerArray * marker_array, int * id, double lambda);

  visualization_msgs::Marker createNodeMarker(RRTNode * node, int id, std::string frame_id);
  visualization_msgs::Marker createEdgeMarker(RRTNode * node, int id, std::string frame_id, double lambda);

  void visualizePose(ros::Publisher& marker_pub, const int& object_id, const geometry_msgs::Pose& pose);
  
  void visualizePrediction(ros::Publisher& line_marker_pub, 
                                    ros::Publisher& ellipse_marker_pub, 
                                    const std::vector<std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>>& trajectories,
                                    const std::vector<std::vector<std::tuple<double, double, Eigen::MatrixXd>>>& all_ellipses
                                    );
  void visualizeBestNode(ros::Publisher& best_node_pub, std::vector<Eigen::Vector4d> nodes);

  void visualizeGhostPedestrian(ros::Publisher& ghost_marker_pub, std::vector<std::tuple<double, double, double>> positions);

  visualization_msgs::Marker createRayMarker(int ray_id, std::string color);

  void VisualizeOldPath(ros::Publisher& old_path_pub, RRTNode* root);
  

}
#endif
