#include <aeplanner/aeplanner_viz.h>

namespace aeplanner {
  Params params_ = readParams();

visualization_msgs::Marker createRayMarker(int ray_id, std::string color)
{
    // Visualize trajectory
    visualization_msgs::Marker ray;
    ray.header.frame_id = "world";
    ray.header.stamp = ros::Time::now();
    ray.ns = "ray";
    ray.action = visualization_msgs::Marker::ADD;
    ray.type = visualization_msgs::Marker::LINE_STRIP;
    ray.id = ray_id;
    ray.scale.x = 0.05;  
    if(color == "BLUE")
    {
      ray.color.r = 0.0;  
      ray.color.g = 0.0;  
      ray.color.b = 1.0;  
    }
    else
    {
      //Must be RED
      ray.color.r = 1.0;  
      ray.color.g = 0.0;  
      ray.color.b = 0.0;  
    }
    ray.color.a = 1.0; 
    ray.pose.orientation.x = 0.0;
    ray.pose.orientation.y = 0.0;
    ray.pose.orientation.z = 0.0;
    ray.pose.orientation.w = 1.0;
    return ray;
}

/*
Visualizes the old best branch
*/
void VisualizeOldPath(ros::Publisher& old_path_pub, RRTNode* root) {
    // Create a marker array message
    visualization_msgs::MarkerArray markerArray;
    
    // Create a line strip marker
    visualization_msgs::Marker line_strip_marker;
    line_strip_marker.header.frame_id = "world";  
    line_strip_marker.header.stamp = ros::Time::now();   
    line_strip_marker.ns = "rrt_line_strip";
    line_strip_marker.action = visualization_msgs::Marker::ADD;
    line_strip_marker.pose.orientation.w = 1.0;           
    line_strip_marker.id = 0;
    line_strip_marker.type = visualization_msgs::Marker::LINE_STRIP;
    line_strip_marker.scale.x = 0.05;                     
    line_strip_marker.color.r = 0.961;      
    line_strip_marker.color.g = 0.596;  
    line_strip_marker.color.b = 0.137;             
    line_strip_marker.color.a = 1.0;                  
    
    // Loop until root
    RRTNode* curr = root;
    while (!curr->children_.empty()) {
        // Add the state point to the line strip marker
        geometry_msgs::Point point;
        point.x = curr->state_(0);
        point.y = curr->state_(1);
        point.z = curr->state_(2);
        line_strip_marker.points.push_back(point);
        
        // Move to the child node
        curr = curr->children_[0];
    }
    
    // Add the line strip marker to the marker array
    markerArray.markers.push_back(line_strip_marker);
    
    // Publish the marker array
    old_path_pub.publish(markerArray);
}


/**
 * Visualize the ghost pedestrian that will block our view in the future.
*/
void visualizeGhostPedestrian(ros::Publisher& ghost_marker_pub, std::vector<std::tuple<double, double, double>> positions) 
{
    
    visualization_msgs::MarkerArray ghosts;
    int id = 0;
    for(auto const& pos : positions)
    {
      // Create visualization marker
      visualization_msgs::Marker ghost;
      ghost.header.frame_id = "world";  
      ghost.header.stamp = ros::Time::now();
      ghost.ns = "dynamic_objects";
      ghost.id = id++;
      ghost.type = visualization_msgs::Marker::CUBE;
      ghost.action = visualization_msgs::Marker::ADD;
      ghost.pose.position.x = std::get<0>(pos);
      ghost.pose.position.y = std::get<1>(pos);
      ghost.pose.position.z = std::get<2>(pos)  + (params_.human_height/2);
      ghost.scale.x = params_.human_width;  
      ghost.scale.y = params_.human_width;  
      ghost.scale.z = params_.human_height; 

      //GREY
      ghost.color.r = 0.769;  
      ghost.color.g = 0.769;  
      ghost.color.b = 0.769;
      ghost.color.a = 1.0;

      ghost.pose.orientation.x = 0.0;
      ghost.pose.orientation.y = 0.0;
      ghost.pose.orientation.z = 0.0;
      ghost.pose.orientation.w = 1.0;

      ghosts.markers.push_back(ghost);
    }
    
    // Publish the marker
    ghost_marker_pub.publish(ghosts);
}

/**
 * This function visualizes the mean trajectory (line) for each dynamic obstacle
 * as well as the corresponding covariance ellipses corresponding to the mean trajectory.
 * This visualization can be seen in RViz.
*/
void visualizePrediction(ros::Publisher& line_marker_pub, 
                         ros::Publisher& ellipse_marker_pub, 
                         const std::vector<std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>>& trajectories,
                         const std::vector<std::vector<std::tuple<double, double, Eigen::MatrixXd>>>& all_ellipses
                        ) 
{
    visualization_msgs::MarkerArray line_marker_array;
    visualization_msgs::MarkerArray ellipse_marker_array;
    int line_id = 0; // Line Marker ID counter
    int ellipse_id = 0; // Ellipse Marker ID counter
    int N_persons = all_ellipses.size();

    for (int i = 0; i < N_persons; i++){

      // Visualize trajectory
      visualization_msgs::Marker line_marker;
      line_marker.header.frame_id = "world";
      line_marker.header.stamp = ros::Time::now();
      line_marker.ns = "predicted_position";
      line_marker.action = visualization_msgs::Marker::ADD;
      line_marker.type = visualization_msgs::Marker::LINE_STRIP;
      line_marker.id = line_id++;
      line_marker.scale.x = 0.1;  
      line_marker.color.r = 0.5;  
      line_marker.color.g = 0.4;  
      line_marker.color.b = 0.4;  
      line_marker.color.a = 1.0; 
      line_marker.pose.orientation.x = 0.0;
      line_marker.pose.orientation.y = 0.0;
      line_marker.pose.orientation.z = 0.0;
      line_marker.pose.orientation.w = 1.0;

      // Extract trajectory and ellipses for a certain dynamic obstacle (person)
      const std::vector<double>& xCoords = std::get<0>(trajectories[i]);
      const std::vector<double>& yCoords = std::get<1>(trajectories[i]);
      const std::vector<double>& zCoords = std::get<2>(trajectories[i]);
      
      const std::vector<std::tuple<double, double, Eigen::MatrixXd>>& ellipses = all_ellipses[i]; //List of tuples

      int N_trajectory_steps = xCoords.size();
      
      for (int j = 0; j < N_trajectory_steps; ++j) {
         
          // Visualize trajectory by filling a line with points
          geometry_msgs::Point point;
          point.x = xCoords[j];
          point.y = yCoords[j];
          point.z = zCoords[j] + (params_.human_height/2);
          line_marker.points.push_back(point);

          //Extract the values to build the ellipse
          double major_length = std::get<0>(ellipses[j]);
          double minor_length = std::get<1>(ellipses[j]);
          Eigen::MatrixXd eigenvectors = std::get<2>(ellipses[j]);

          //Visualize Ellipse
          visualization_msgs::Marker ellipse_marker;
          ellipse_marker.header.frame_id = "world";
          ellipse_marker.header.stamp = ros::Time::now();
          ellipse_marker.id = ellipse_id++;
          ellipse_marker.type = visualization_msgs::Marker::SPHERE;
          ellipse_marker.pose.position.x = xCoords[j];
          ellipse_marker.pose.position.y = yCoords[j];
          ellipse_marker.pose.position.z = zCoords[j] + 0.2;
          ellipse_marker.color.a = 1.0;
          ellipse_marker.color.r = 0.0;
          ellipse_marker.color.g = 1.0;
          ellipse_marker.color.b = 0.0;

          // Compute orientation quaternion of covariance ellipse
          Eigen::Quaterniond q(Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), Eigen::Vector3d(eigenvectors(0), eigenvectors(1), 0.0)));
          q.x() = 0;
          q.y() = 0;
          q.normalize();
          ellipse_marker.pose.orientation.x = q.x();
          ellipse_marker.pose.orientation.y = q.y();
          ellipse_marker.pose.orientation.z = q.z();
          ellipse_marker.pose.orientation.w = q.w();

          // Give the marker the right dimensions according to the major and minor length
          ellipse_marker.scale.x = 2.0 * major_length;
          ellipse_marker.scale.y = 2.0 * minor_length;
          ellipse_marker.scale.z = 0.1;
          
          ellipse_marker_array.markers.push_back(ellipse_marker);
          
      }
      line_marker_array.markers.push_back(line_marker);    
      
      }      
    // Publish the marker arrays (all dynamic obstacles)
    line_marker_pub.publish(line_marker_array);
    ellipse_marker_pub.publish(ellipse_marker_array);
}

/**
 * Visualize the ground truth for each dynamic obstacle in RViz as a cylinder.
*/
void visualizePose(ros::Publisher& marker_pub, const int& object_id, const geometry_msgs::Pose& pose) {
    // Create visualization marker
    visualization_msgs::Marker cylinder_marker;
    cylinder_marker.header.frame_id = "world";  
    cylinder_marker.header.stamp = ros::Time::now();
    cylinder_marker.ns = "dynamic_objects";
    cylinder_marker.id = object_id;
    cylinder_marker.type = visualization_msgs::Marker::CYLINDER;
    cylinder_marker.action = visualization_msgs::Marker::ADD;
    cylinder_marker.pose = pose;
    cylinder_marker.scale.x = params_.human_width; 
    cylinder_marker.scale.y = params_.human_width;  
    cylinder_marker.scale.z = params_.human_height; 
    cylinder_marker.color.r = 1.0;  
    cylinder_marker.color.g = 1.0;  
    cylinder_marker.color.b = 1.0;  
    cylinder_marker.color.a = 1.0;  
    cylinder_marker.pose.position.z = pose.position.z + params_.human_height/2;

    // Publish the marker
    marker_pub.publish(cylinder_marker);
}


  visualization_msgs::MarkerArray createRRTMarkerArray(RRTNode * root, double lambda){
    int id = 0;
    visualization_msgs::MarkerArray marker_array;
    recurse(root, &marker_array, &id, lambda);

    return marker_array;
  }
  
  void recurse(RRTNode * node, visualization_msgs::MarkerArray * marker_array, int * id, double lambda){
    for(std::vector<RRTNode*>::iterator child_it  = node->children_.begin();
                                        child_it != node->children_.end(); ++child_it){
      RRTNode * child = (*child_it);
      if(child) recurse(child, marker_array, id, lambda);
      marker_array->markers.push_back(createEdgeMarker(child, (*id), "world", lambda));
      marker_array->markers.push_back(createNodeMarker(child, (*id)++, "world"));
    }
  }

  visualization_msgs::Marker createNodeMarker(RRTNode * node, int id, std::string frame_id){
    visualization_msgs::Marker marker;
    marker.header.stamp = ros::Time::now();
    marker.header.seq = id;
    marker.header.frame_id = frame_id;
    marker.id = id;
    marker.ns = "nodes";
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = node->state_[0];
    marker.pose.position.y = node->state_[1];
    marker.pose.position.z = node->state_[2];
    tf::Quaternion quat;
    quat.setEuler(0.0, 0.0, node->state_[3]);
    marker.pose.orientation.x = quat.x();
    marker.pose.orientation.y = quat.y();
    marker.pose.orientation.z = quat.z();
    marker.pose.orientation.w = quat.w();
    marker.scale.x = std::max(node->gain_ / 72.0, 0.05);
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.r = 167.0 / 255.0;
    marker.color.g = 167.0 / 255.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration(10.0);
    marker.frame_locked = false;

    return marker;
  }

  visualization_msgs::Marker createEdgeMarker(RRTNode * node, int id, std::string frame_id, double lambda){
    visualization_msgs::Marker marker;
    marker.header.stamp = ros::Time::now();
    marker.header.seq = id;
    marker.header.frame_id = frame_id;
    marker.id = id;
    marker.ns = "edges";
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = node->parent_->state_[0];
    marker.pose.position.y = node->parent_->state_[1];
    marker.pose.position.z = node->parent_->state_[2];
    Eigen::Quaternion<double> q;
    Eigen::Vector3d init(1.0, 0.0, 0.0);
    Eigen::Vector3d dir(node->state_[0] - node->parent_->state_[0],
                        node->state_[1] - node->parent_->state_[1],
                        node->state_[2] - node->parent_->state_[2]);
    q.setFromTwoVectors(init, dir);
    q.normalize();
    marker.pose.orientation.x = q.x();
    marker.pose.orientation.y = q.y();
    marker.pose.orientation.z = q.z();
    marker.pose.orientation.w = q.w();
    marker.scale.x = dir.norm();
    marker.scale.y = 0.03;
    marker.scale.z = 0.03;
    marker.color.r = node->score(lambda) / 60.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration(10.0);
    marker.frame_locked = false;

    return marker;
  }


/*
Visualizes the best node and the other sampled nodes. It highlights the best node as green, 
valid nodes as red, and collision nodes as black.
*/
  void visualizeBestNode(ros::Publisher& best_node_pub, std::vector<Eigen::Vector4d> nodes)
{
  
  // Clear the current markers because of different amount of nodes
  visualization_msgs::MarkerArray clearMarkerArray;
  visualization_msgs::Marker clearMarker;
  clearMarker.header.frame_id = "world";
  clearMarker.header.stamp = ros::Time::now();  
  clearMarker.ns = "cube";  
  clearMarker.type = visualization_msgs::Marker::CUBE;  
  clearMarker.action = visualization_msgs::Marker::DELETEALL;
  clearMarkerArray.markers.push_back(clearMarker);
  best_node_pub.publish(clearMarkerArray);

  visualization_msgs::MarkerArray markerArray;
  int idd = 0;

  for(auto node : nodes){
    
    auto probability = node(3);
    
    // Create a marker message
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";  
    marker.header.stamp = ros::Time::now(); 
    marker.ns = "cube";  
    marker.id = idd++;  
    marker.type = visualization_msgs::Marker::CUBE;  
    marker.action = visualization_msgs::Marker::ADD; 
    marker.pose.position.x = node(0);  
    marker.pose.position.y = node(1);  
    marker.pose.position.z = node(2);  
    marker.pose.orientation.x = 0.0;  
    marker.pose.orientation.y = 0.0;  
    marker.pose.orientation.z = 0.0;  
    marker.pose.orientation.w = 1.0; 
    marker.scale.x = 0.5;  
    marker.scale.y = 0.5;  
    marker.scale.z = 0.5;  
    if (probability == 2){
      marker.color.r = 0.2;  
      marker.color.g = 0.8;  
      marker.color.b = 0.0;  
      marker.color.a = 1.0;  
      marker.scale.x = 0.7;  
      marker.scale.y = 0.7;  
      marker.scale.z = 0.7;  
      }else {
      marker.color.r = 1.0-probability;  
      marker.color.g = 0.0;  
      marker.color.b = 0.0;  
      marker.color.a = 1.0;  
    }
    markerArray.markers.push_back(marker);
  }
  best_node_pub.publish(markerArray);
}

}

