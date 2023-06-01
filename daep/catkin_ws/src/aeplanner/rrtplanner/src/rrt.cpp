#include <rrtplanner/rrt.h>
#include <tf2/utils.h>
#include <cmath>

namespace aeplanner_ns
{

Rrt::Rrt(const ros::NodeHandle& nh)
  : nh_(nh)
  , frame_id_("world")
  , path_pub_(nh_.advertise<visualization_msgs::Marker>("rrt_path", 1000))
  , octomap_sub_(nh_.subscribe("octomap", 1, &Rrt::octomapCallback, this))
  , human_sub_(nh_.subscribe("/gazebo/model_states", 1, &Rrt::updateHumanPositions, this)) // DAEP
  , dynamic_mode_(false)
  , safe_path_srv(nh_.advertiseService("/rrtplanner/safe_path", &Rrt::safePathSrvCallback, this))
  , as_(nh_, "rrt", boost::bind(&Rrt::execute, this, _1), false)
{

  std::string ns = ros::this_node::getNamespace();
  bounding_radius_ = 0.5;
  bounding_overshoot_ = 0.5;
  extension_range_ = 1.0;
  min_nodes_ = 100;
  dt_ = 0.5;
  KFiterations_ = 15;
  drone_linear_velocity = 0.5;
  drone_angular_velocity = 1;


  if (!ros::param::get(ns + "/rrt/min_nodes", min_nodes_))
    ROS_WARN("No minimum nodes specified default is 100");
  if (!ros::param::get(ns + "/system/bbx/r", bounding_radius_))
    ROS_WARN("No bounding radius specified default is 0.5 m");
  if (!ros::param::get(ns + "/system/bbx/overshoot", bounding_overshoot_))
    ROS_WARN("No overshoot paramerer specified, default is 0.5 m");
  //if (!ros::param::get(ns + "/aep/tree/extension_range", extension_range_))
  //  ROS_WARN("No extension range specified, default is 1.0 m");
  if (!ros::param::get(ns + "/boundary/min", boundary_min_))
    ROS_WARN("No boundary min specified.");
  if (!ros::param::get(ns + "/boundary/max", boundary_max_))
    ROS_WARN("No boundary max specified.");
  if (!ros::param::get(ns + "/daep/kf/time_step", dt_)) {
  ROS_WARN_STREAM("No /daep/kf/time_step specified. Default: " << dt_);
  }
  if (!ros::param::get(ns + "/daep/kf/iterations", KFiterations_)) {
    ROS_WARN_STREAM("No /daep/kf/iterations specified. Default: " << KFiterations_);
  }
  if (!ros::param::get(ns + "/drone_linear_velocity", drone_linear_velocity)) {
    ROS_WARN_STREAM("No /drone_linear_velocity specified. Default: " << drone_linear_velocity);
  }
  if (!ros::param::get(ns + "/drone_angular_velocity", drone_angular_velocity)) {
    ROS_WARN_STREAM("No /drone_angular_velocity specified. Default: " << drone_angular_velocity);
  }
 
  ot_ = std::make_shared<octomap::OcTree>(
      1);  // Create dummy OcTree to prevent crash due to ot_ tree not initialized
  as_.start();
}


bool Rrt::safePathSrvCallback(rrtplanner::SafePath::Request& request, rrtplanner::SafePath::Response& response)
{

  auto predicted_data = KFpredictTrajectories();

  for(int i = 0; i < request.path.poses.size(); i++)
  {
    if(i == request.path.poses.size() - 1)
    {
      //Avoid to index outside of path
      break;
    }

    if(isCollision(request.path.poses[i], request.path.poses[i + 1], request.time_steps[i], predicted_data))
    {
      response.safe = false;
      return true;
    }
  }
  response.safe = true;
  return true;
}


bool Rrt::isCollision(const geometry_msgs::PoseStamped& posestamped_parent, const geometry_msgs::PoseStamped& posestamped, double time_of_arrival,
                      std::vector<std::pair<std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>, std::vector<Eigen::MatrixXd>>> predicted_data)
{

  std::vector<std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>> trajectories{};
  std::vector<std::vector<std::tuple<double, double, Eigen::MatrixXd>>> all_ellipses{};

  //Extract each person's trajectory and covariance trajectory
  for(auto const& person : predicted_data)
  {
    trajectories.push_back(person.first);
    all_ellipses.push_back(createCovarianceEllipse(person.second));
  }

  Eigen::Vector3d next_pose(posestamped.pose.position.x, 
                            posestamped.pose.position.y, 
                            posestamped.pose.position.z);

  Eigen::Vector3d next_pose_parent(posestamped_parent.pose.position.x, 
                                   posestamped_parent.pose.position.y, 
                                   posestamped_parent.pose.position.z);

  bool collision = checkCollision(time_of_arrival, next_pose, 
                                  trajectories, all_ellipses, next_pose_parent);

  return collision;
}


/**
 * Subscribe on the gazebo/model_states topic to extract the positions of the 
 * dynamic obstacles. Add these to the dynamic_objects std::map and visualzie 
 * the ground truth in RViz.
*/
void Rrt::updateHumanPositions(const gazebo_msgs::ModelStates& model_states) 
{
    int human_id = 0;
    for (size_t i = 0; i < model_states.name.size(); ++i) {
        if (model_states.name[i].find("person_walking") != std::string::npos) {
            dynamic_mode_ = true;
            std::string model_name = model_states.name[i];
            geometry_msgs::Pose person_pose = model_states.pose[i];
            geometry_msgs::Twist person_twist = model_states.twist[i];
            dynamic_objects[model_name] = std::make_pair(person_pose, person_twist);
        }
        human_id++;
    }
} 

/**
* This function uses the Kalman Filter with the Constant Velocity
* motion model to predict the future trajectory of each dynamic obstacle.
* The returned data consists of the means and covariances for each dynamic obstacle in a trajectory.
*/
std::vector<std::pair<std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>, std::vector<Eigen::MatrixXd>>> Rrt::KFpredictTrajectories()
{
  std::vector<std::pair<std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>, std::vector<Eigen::MatrixXd>>> kf_data{};

  //For each pedestrian, we need to build a new Kalman filter
  int n = 4; //Number of states
  int m = 2; //Number of states we measure

  //Matrices for the Kalman filter
  Eigen::MatrixXd A(n, n); // System dynamics matrix
  Eigen::MatrixXd C(m, n); // Output matrix
  Eigen::MatrixXd Q(n, n); // Process noise covariance
  Eigen::MatrixXd R(m, m); // Measurement noise covariance
  Eigen::MatrixXd P(n, n); // Estimate error covariance

  //Constant Velocity Model
  A <<  1, 0, dt_, 0, 
        0, 1, 0, dt_, 
        0, 0, 1, 0, 
        0, 0, 0, 1;

  C << 1, 0, 0, 0, 0, 1, 0, 0;

  //Reasonable covariance matrices
  Q << 1, 0, 0, 0,
       0, 1, 0, 0,
       0, 0, 1, 0,
       0, 0, 0, 1;
  
  R << 0.01, 0,
       0, 0.01;
  
  P << 1, 0, 0, 0,
       0, 1, 0, 0,
       0, 0, 1, 0,
       0, 0, 0, 1; 

  Eigen::VectorXd y(m);
  Eigen::VectorXd x0(n);
  Eigen::MatrixXd P0(n, n);
  double t = 0;
  int safety_iteration = 0; // Bounding covariance (1 second)
  
  for (const auto& dynamic_obstacle : dynamic_objects) {
      const std::string& key = dynamic_obstacle.first;
      const geometry_msgs::Pose& pose = dynamic_obstacle.second.first;
      const geometry_msgs::Twist& twist = dynamic_obstacle.second.second;

      // Create a new Kalman filter for each dynamic obstacle
      KalmanFilter kf(dt_, A, C, Q, R, P);
      std::vector<double> xcoords{};
      std::vector<double> ycoords{};
      std::vector<double> zcoords{};
      std::vector<Eigen::MatrixXd> covariance_matrices(KFiterations_);

      // Initalize with the last measurement
      double xcoord = pose.position.x, ycoord = pose.position.y, zcoord = pose.position.z;
      double vx = twist.linear.x, vy = twist.linear.y;
      x0 << xcoord, ycoord, vx, vy;
      
      //Initial covariance
      P0 << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1; 

      kf.init(t, x0, P0);

      //Perform one round of the Kalman filter
      y << xcoord, ycoord;
      kf.predict();
      kf.update(y);

      //Create the trajectory for the current dynamic obstacle
      Eigen::MatrixXd safety_margin(n, n);
      
      for (int i = 0; i < KFiterations_; i++)
      {
        kf.predict();
        xcoords.push_back(kf.state()(0));
        ycoords.push_back(kf.state()(1));
        zcoords.push_back(zcoord);
        if(i == safety_iteration)
        {
          //Because of controllability we can only extract one of these
          //ellipses. See paper for explanation.
          safety_margin = kf.covariance();
        }
      }

      std::fill(covariance_matrices.begin(), covariance_matrices.end(), safety_margin);  
      kf_data.push_back(std::make_pair(std::make_tuple(xcoords, ycoords, zcoords), covariance_matrices)); 
  }
    return kf_data;
}

void Rrt::execute(const rrtplanner::rrtGoalConstPtr& goal)
{
  rrtplanner::rrtResult result;
  if (!ot_)
  {
    ROS_WARN("No octomap received");
    as_.setSucceeded(result);
    return;
  }
  if (!goal->goal_poses.poses.size())
  {
    ROS_WARN("No goals received");
    as_.setSucceeded(result);
    return;
  }

  //Get predictions of dynamic obstacles
  auto predicted_data = KFpredictTrajectories();


  
  std::vector<std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>> trajectories{};
  std::vector<std::vector<std::tuple<double, double, Eigen::MatrixXd>>> all_ellipses{};

   for(auto const& person : predicted_data)
  {
    trajectories.push_back(person.first);
    all_ellipses.push_back(createCovarianceEllipse(person.second));
  }

  int N = min_nodes_;
  double l = extension_range_;
  double r = bounding_radius_;
  double r_os = bounding_overshoot_;
  std::vector<RrtNode*> found_goals;

  kdtree* kd_tree = kd_create(3);    // Initalize tree
  kdtree* goal_tree = kd_create(3);  // kd tree with all goals
  for (int i = 0; i < goal->goal_poses.poses.size(); ++i)
  {
    Eigen::Vector3d* g = new Eigen::Vector3d(goal->goal_poses.poses[i].position.x,
                                             goal->goal_poses.poses[i].position.y,
                                             goal->goal_poses.poses[i].position.z);
    kd_insert3(goal_tree, (*g)[0], (*g)[1], (*g)[2], g);
  }


  // Initialize root position
  RrtNode* root = new RrtNode;
  root->pos[0] = goal->start.pose.position.x;
  root->pos[1] = goal->start.pose.position.y;
  root->pos[2] = goal->start.pose.position.z;
  root->parent = NULL;
  kd_insert3(kd_tree, root->pos[0], root->pos[1], root->pos[2], root);

  
  for (int i = 0; i < N; ++i)
  {
    // Sample new position
    Eigen::Vector3d sampled_node = sample();
    // Get nearest neighbour
    RrtNode* nearest = chooseParent(kd_tree, sampled_node, l);
    if (!nearest)
      continue;

    // Calculate position for new node
    Eigen::Vector3d normalised_node = getNewPosNormalized(sampled_node, nearest->pos, l);
    Eigen::Vector3d direction = normalised_node - nearest->pos;
    
  //###################### DAEP ########################
    // Estimate time to reach node
    double time_to_reach_node = nearest->time_cost(drone_linear_velocity, drone_angular_velocity) + nearest->time_to_reach(normalised_node,drone_linear_velocity, drone_angular_velocity);

    // Check if the sampled node will collide in the future
    bool collision = checkCollision(time_to_reach_node, normalised_node, 
                                    trajectories, all_ellipses, nearest->pos);

  //###################### END DAEP #####################

    if(!collisionLine(nearest->pos, normalised_node + direction.normalized() * r_os, r) and !collision)
    {
      // Add node to tree
      RrtNode* valid_node = addNodeToTree(kd_tree, nearest, normalised_node);
      rewire(kd_tree, valid_node, l, r, r_os);
      visualizeEdge(valid_node, i);
      // Check if goal has been reached
      RrtNode* tmp_goal = getGoal(goal_tree, valid_node, l, r, r_os);
      if (tmp_goal)
      {
        found_goals.push_back(tmp_goal);
      }
    }
    else
    {
      --i;
    }
  }

  std::pair<nav_msgs::Path, std::vector<double>> path_and_time = getBestPath(found_goals);

  result.path = path_and_time.first;
  result.time_steps = path_and_time.second;

  delete root;
  kd_free(kd_tree);
  kd_free(goal_tree);

  as_.setSucceeded(result);
}

void Rrt::octomapCallback(const octomap_msgs::Octomap& msg)
{
  octomap::AbstractOcTree* aot = octomap_msgs::msgToMap(msg);
  octomap::OcTree* ot = (octomap::OcTree*)aot;
  ot_ = std::make_shared<octomap::OcTree>(*ot);

  if (ot)
    delete ot;
}


Eigen::Vector3d Rrt::sample()
{
  Eigen::Vector3d x_samp;
  for (int i = 0; i < 3; ++i)
  {
    x_samp[i] = boundary_min_[i] + (((double)rand()) / ((double)RAND_MAX)) * (boundary_max_[i] - boundary_min_[i]);
  }

  return x_samp;
}



RrtNode* Rrt::chooseParent(kdtree* kd_tree, Eigen::Vector3d node, double l)
{
  kdres* nearest = kd_nearest_range3(kd_tree, node[0], node[1], node[2], l + 0.5);
  if (kd_res_size(nearest) <= 0)
  {
    nearest = kd_nearest3(kd_tree, node[0], node[1], node[2]);
  }
  if (kd_res_size(nearest) <= 0)
  {
    kd_res_free(nearest);
    return NULL;
  }

  RrtNode* node_nn = (RrtNode*)kd_res_item_data(nearest);
  int i = 0;

  RrtNode* best_node = node_nn;
  while (!kd_res_end(nearest))
  {
    node_nn = (RrtNode*)kd_res_item_data(nearest);
    if (best_node and node_nn->cost() < best_node->cost())
      best_node = node_nn;

    kd_res_next(nearest);
  }

  kd_res_free(nearest);
  return best_node;
}

void Rrt::rewire(kdtree* kd_tree, RrtNode* new_node, double l, double r, double r_os)
{
  RrtNode* node_nn;
  kdres* nearest = kd_nearest_range3(kd_tree, new_node->pos[0], new_node->pos[1],
                                     new_node->pos[2], l + 0.5);
  while (!kd_res_end(nearest))
  {
    node_nn = (RrtNode*)kd_res_item_data(nearest);
    if (node_nn->cost() > new_node->cost() + (node_nn->pos - new_node->pos).norm())
    {
      if (!collisionLine(
              new_node->pos,
              node_nn->pos + (node_nn->pos - new_node->pos).normalized() * r_os, r))
        node_nn->parent = new_node;
    }
    kd_res_next(nearest);
  }
}

Eigen::Vector3d Rrt::getNewPosNormalized(Eigen::Vector3d sampled, Eigen::Vector3d parent,
                               double l)
{
  Eigen::Vector3d direction = sampled - parent;
  if (direction.norm() > l)
    direction = l * direction.normalized();

  return parent + direction;
}

RrtNode* Rrt::addNodeToTree(kdtree* kd_tree, RrtNode* parent,
                            Eigen::Vector3d new_pos)
{
  RrtNode* new_node = new RrtNode;
  new_node->pos = new_pos;

  new_node->parent = parent;
  parent->children.push_back(new_node);
  kd_insert3(kd_tree, new_node->pos[0], new_node->pos[1], new_node->pos[2],
             new_node);

  return new_node;
}

RrtNode* Rrt::getGoal(kdtree* goal_tree, RrtNode* new_node, double l, double r,
                      double r_os)
{
  kdres* nearest_goal =
      kd_nearest3(goal_tree, new_node->pos[0], new_node->pos[1], new_node->pos[2]);
  if (kd_res_size(nearest_goal) <= 0)
  {
    kd_res_free(nearest_goal);
    return NULL;
  }
  Eigen::Vector3d* g_nn = (Eigen::Vector3d*)kd_res_item_data(nearest_goal);
  kd_res_free(nearest_goal);

  if ((*g_nn - new_node->pos).norm() < 1.5)
    if (!collisionLine(new_node->pos,
                       *g_nn + (*g_nn - new_node->pos).normalized() * r_os, r))
      return new_node;

  return NULL;
}

std::pair<nav_msgs::Path, std::vector<double>> Rrt::getBestPath(std::vector<RrtNode*> goals)
{

  std::vector<double> time_steps{};
  nav_msgs::Path path;

  if (goals.size() == 0)
  {
    return std::make_pair(path, time_steps);
  }

  RrtNode* best_node = goals[0];

  for (int i = 0; i < goals.size(); ++i)
    if (best_node->cost() > goals[i]->cost())
      best_node = goals[i];

  RrtNode* n = best_node;

  //Extract path to the best node
  for (int id = 0; n->parent; ++id)
  {
    geometry_msgs::PoseStamped p;
    p.pose.position.x = n->pos[0];
    p.pose.position.y = n->pos[1];
    p.pose.position.z = n->pos[2];
    Eigen::Quaternion<double> q;
    Eigen::Vector3d init(1.0, 0.0, 0.0);
    // Zero out rotation along
    // x and y axis so only
    // yaw is kept
    Eigen::Vector3d dir(n->pos[0] - n->parent->pos[0], n->pos[1] - n->parent->pos[1],
                        0);
    q.setFromTwoVectors(init, dir);

    p.pose.orientation.x = q.x();
    p.pose.orientation.y = q.y();
    p.pose.orientation.z = q.z();
    p.pose.orientation.w = q.w();

    path.poses.push_back(p);
    time_steps.push_back(n->time_cost(drone_linear_velocity, drone_angular_velocity)); //Add time it takes to reach this node from root
    n = n->parent;
  }

  std::reverse(time_steps.begin(), time_steps.end());
  std::reverse(path.poses.begin(), path.poses.end());

  visualizePath(best_node);
  return std::make_pair(path, time_steps);
}

std::vector<geometry_msgs::Pose> Rrt::checkIfGoalReached(kdtree* goal_tree,
                                                         RrtNode* new_node, double l,
                                                         double r, double r_os)
{
  std::vector<geometry_msgs::Pose> path;

  kdres* nearest_goal =
      kd_nearest3(goal_tree, new_node->pos[0], new_node->pos[1], new_node->pos[2]);
  if (kd_res_size(nearest_goal) <= 0)
  {
    kd_res_free(nearest_goal);
    return path;
  }
  Eigen::Vector3d* g_nn = (Eigen::Vector3d*)kd_res_item_data(nearest_goal);
  kd_res_free(nearest_goal);

  if ((*g_nn - new_node->pos).norm() < 2 * l)
  {
    if (!collisionLine(new_node->pos,
                       *g_nn + (*g_nn - new_node->pos).normalized() * r_os, r))
    {
      RrtNode* n = new_node;
      for (int id = 0; n->parent; ++id)
      {
        geometry_msgs::Pose p;
        p.position.x = n->pos[0];
        p.position.y = n->pos[1];
        p.position.z = n->pos[2];
        Eigen::Quaternion<double> q;
        Eigen::Vector3d init(1.0, 0.0, 0.0);
        // Zero out rotation
        // along x and y axis
        // so only yaw is kept
        Eigen::Vector3d dir(n->pos[0] - n->parent->pos[0],
                            n->pos[1] - n->parent->pos[1], 0);
        q.setFromTwoVectors(init, dir);

        p.orientation.x = q.x();
        p.orientation.y = q.y();
        p.orientation.z = q.z();
        p.orientation.w = q.w();

        path.push_back(p);
        visualizePose(p, id);

        n = n->parent;
      }

      visualizePath(new_node);
    }
  }

  return path;
}

bool Rrt::collisionLine(Eigen::Vector3d p1, Eigen::Vector3d p2, double r)
{
  std::shared_ptr<octomap::OcTree> ot = ot_;

  octomap::point3d start(p1[0], p1[1], p1[2]);
  octomap::point3d end(p2[0], p2[1], p2[2]);
  octomap::point3d min(std::min(p1[0], p2[0]) - r, std::min(p1[1], p2[1]) - r,
                       std::min(p1[2], p2[2]) - r);
  octomap::point3d max(std::max(p1[0], p2[0]) + r, std::max(p1[1], p2[1]) + r,
                       std::max(p1[2], p2[2]) + r);
  double lsq = (end - start).norm_sq();
  double rsq = r * r;

  octomap::point3d query(p2[0], p2[1], p2[2]);
  octomap::OcTreeNode* ot_res = ot->search(query);
  if (!ot_res)
    return true;

  for (octomap::OcTree::leaf_bbx_iterator it = ot->begin_leafs_bbx(min, max),
                                          it_end = ot->end_leafs_bbx();
       it != it_end; ++it)
  {
    octomap::point3d pt(it.getX(), it.getY(), it.getZ());

    if (it->getLogOdds() > 0)
    {  // Node is occupied
      if (CylTest_CapsFirst(start, end, lsq, rsq, pt) > 0 or (end - pt).norm() < r)
      {
        return true;
      }
    }
  }
  return false;
}

/**
 * From a vector of covariance matrices, compute a list of tuples that
 * represent each covariance circle.
 * 
 * Return: (major_lenght, minor_length, eigenvectors) 
*/
std::vector<std::tuple<double, double, Eigen::MatrixXd>> Rrt::createCovarianceEllipse(const std::vector<Eigen::MatrixXd>& cov_matrices)
{
  
  std::vector<std::tuple<double, double, Eigen::MatrixXd>> ellipses{};

  for(auto const& cov_matrix : cov_matrices)
  {
    //Extract the part concerning x,y
    Eigen::MatrixXd cov_matrix_xy = cov_matrix.block<2,2>(0,0);

    // Compute the eigenvalues and eigenvectors of the covariance matrix
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigen_solver(cov_matrix_xy);
    Eigen::VectorXd eigenvalues = eigen_solver.eigenvalues();
    Eigen::MatrixXd eigenvectors = eigen_solver.eigenvectors();

    // Compute the length of the major and minor axes of the ellipse
    double major_length = std::sqrt(std::max(eigenvalues(0), eigenvalues(1)));
    double minor_length = std::sqrt(std::min(eigenvalues(0), eigenvalues(1)));

    ellipses.push_back(std::make_tuple(major_length, minor_length, eigenvectors));
  }
  return ellipses;
}

/* 
* Calculate if there is a potential collision with any moving obstacle in time t.
*/
bool Rrt::checkCollision(double t, 
                          Eigen::Vector3d point, 
                          std::vector<std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>> trajectories, 
                          std::vector<std::vector<std::tuple<double, double, Eigen::MatrixXd>>> covarianceEllipses,
                          Eigen::Vector3d parent)
{
  double max_time_step = KFiterations_ * dt_;
  int N_persons = trajectories.size();
  
  // Index the correct circle
  int covariance_index = getCovarianceIndex(max_time_step, dt_, t);

  if(covariance_index == -1)
  { 
    // Outside our prediction
    return false;
  }

  for(int i = 0; i < N_persons; i++)
  { 
      auto personEllipses = covarianceEllipses[i];
      auto personTrajectory = trajectories[i];
      
      //Extract the correct mean and ellipse
      std::tuple<double, double, Eigen::MatrixXd> covarianceEllipse = personEllipses[covariance_index];
      double ellipse_center_x = std::get<0>(personTrajectory)[covariance_index];
      double ellipse_center_y = std::get<1>(personTrajectory)[covariance_index];
      double ellipse_center_z = std::get<2>(personTrajectory)[covariance_index];
      double radius = std::get<0>(covarianceEllipse);

      const Eigen::Vector3d center(ellipse_center_x, ellipse_center_y, ellipse_center_z);
        
      if(lineIntersectsCircle(parent, point, center, radius))
      {
        return true;
      }
  }
  return false;
}


bool Rrt::lineIntersectsCircle(const Eigen::Vector3d p1, const Eigen::Vector3d p2,
                               const Eigen::Vector3d center, double radius)
{
  double dx = p2.x() - p1.x();
  double dy = p2.y() - p1.y();
  double a = dx * dx + dy * dy;
  double b = 2 * (dx * (p1.x() - center.x()) + dy * (p1.y() - center.y()));
  double c = center.x() * center.x() + center.y() * center.y();
  c += p1.x() * p1.x() + p1.y() * p1.y();
  c -= 2 * (center.x() * p1.x() + center.y() * p1.y());
  c -= radius * radius;
  double discriminant = b * b - 4 * a * c;

  if(discriminant >= 0)
  {
    double t1 = (-b - sqrt(discriminant)) / (2*a);
    double t2 = (-b + sqrt(discriminant)) / (2*a);
  
    if((t1 >= 0 and t1 <= 1) or (t2 >= 0 and t2 <= 1))
    {
      return true;
    }
  }
  return false;

}


/*
* Gets the correct covariance ellipse from a prediction given time t.
*/
int Rrt::getCovarianceIndex(double max_time_step, double time_step, double t){
  
  int num_steps = static_cast<int>(max_time_step / time_step); // Calculate the number of time steps

  if(t > max_time_step)
  {
    //No prediction available
    return -1;
  }

  // Initialize the minimum difference and index
  double min_difference = std::abs(t - time_step);
  int index = 0;

  // Loop through each time step and find the closest one
  for (int i = 1; i < num_steps; ++i) {
    double time = i * time_step; // Calculate the time for the current step
    double difference = std::abs(t - time);
    if (difference < min_difference) {
      min_difference = difference;
      index = i;
    }
  }
  return index;
}

//-----------------------------------------------------------------------------
// Name: CylTest_CapsFirst
// Orig: Greg James -
// gjames@NVIDIA.com Lisc:
// Free code - no warranty &
// no money back.  Use it all
// you want Desc:
//    This function tests if
//    the 3D point 'pt' lies
//    within an arbitrarily
// oriented cylinder.  The
// cylinder is defined by an
// axis from 'pt1' to 'pt2',
// the axis having a length
// squared of 'lsq'
// (pre-compute for each
// cylinder to avoid repeated
// work!), and radius squared
// of 'rsq'.
//    The function tests
//    against the end caps
//    first, which is cheap ->
//    only
// a single dot product to
// test against the parallel
// cylinder caps.  If the
// point is within these, more
// work is done to find the
// distance of the point from
// the cylinder axis.
//    Fancy Math (TM) makes
//    the whole test possible
//    with only two
//    dot-products
// a subtract, and two
// multiplies.  For clarity,
// the 2nd mult is kept as a
// divide.  It might be faster
// to change this to a mult by
// also passing in 1/lengthsq
// and using that instead.
//    Elminiate the first 3
//    subtracts by specifying
//    the cylinder as a base
// point on one end cap and a
// vector to the other end cap
// (pass in {dx,dy,dz} instead
// of 'pt2' ).
//
//    The dot product is
//    constant along a plane
//    perpendicular to a
//    vector. The magnitude of
//    the cross product
//    divided by one vector
//    length is
// constant along a cylinder
// surface defined by the
// other vector as axis.
//
// Return:  -1.0 if point is
// outside the cylinder
// Return:  distance squared
// from cylinder axis if point
// is inside.
//
//-----------------------------------------------------------------------------
float CylTest_CapsFirst(const octomap::point3d& pt1, const octomap::point3d& pt2,
                        float lsq, float rsq, const octomap::point3d& pt)
{
  float dx, dy,
      dz;  // vector d  from
           // line segment
           // point 1 to point
           // 2
  float pdx, pdy,
      pdz;  // vector pd from
            // point 1 to test
            // point
  float dot, dsq;

  dx = pt2.x() - pt1.x();  // translate
                           // so pt1 is
                           // origin.
                           // Make vector
                           // from
  dy = pt2.y() - pt1.y();  // pt1 to
                           // pt2.  Need
                           // for this
                           // is easily
                           // eliminated
  dz = pt2.z() - pt1.z();

  pdx = pt.x() - pt1.x();  // vector from
                           // pt1 to test
                           // point.
  pdy = pt.y() - pt1.y();
  pdz = pt.z() - pt1.z();

  // Dot the d and pd vectors
  // to see if point lies
  // behind the cylinder cap
  // at pt1.x, pt1.y, pt1.z

  dot = pdx * dx + pdy * dy + pdz * dz;

  // If dot is less than zero
  // the point is behind the
  // pt1 cap. If greater than
  // the cylinder axis line
  // segment length squared
  // then the point is outside
  // the other end cap at pt2.

  if (dot < 0.0f || dot > lsq)
  {
    return (-1.0f);
  }
  else
  {
    // Point lies within the
    // parallel caps, so find
    // distance squared from
    // point to line, using
    // the fact that sin^2 +
    // cos^2 = 1 the dot =
    // cos() * |d||pd|, and
    // cross*cross = sin^2 *
    // |d|^2 * |pd|^2
    // Carefull: '*' means
    // mult for scalars and
    // dotproduct for vectors
    // In short, where dist is
    // pt distance to cyl
    // axis: dist = sin( pd to
    // d ) * |pd| distsq = dsq
    // = (1 - cos^2( pd to d))
    // * |pd|^2 dsq = ( 1 -
    // (pd * d)^2 / (|pd|^2 *
    // |d|^2) ) * |pd|^2 dsq =
    // pd * pd - dot * dot /
    // lengthsq
    //  where lengthsq is d*d
    //  or |d|^2 that is
    //  passed into this
    //  function

    // distance squared to the
    // cylinder axis:

    dsq = (pdx * pdx + pdy * pdy + pdz * pdz) - dot * dot / lsq;

    if (dsq > rsq)
    {
      return (-1.0f);
    }
    else
    {
      return (dsq);  // return
                     // distance
                     // squared
                     // to
                     // axis
    }
  }
}

void Rrt::visualizeNode(geometry_msgs::Point pos, int id)
{
  visualization_msgs::Marker a;
  a.header.stamp = ros::Time::now();
  a.header.seq = id;
  a.header.frame_id = frame_id_;
  a.id = id;
  a.ns = "nodes";
  a.type = visualization_msgs::Marker::SPHERE;
  a.action = visualization_msgs::Marker::ADD;
  a.pose.position = pos;

  a.scale.x = 0.2;
  a.scale.y = 0.2;
  a.scale.z = 0.2;
  a.color.r = 0.2;
  a.color.g = 0.7;
  a.color.b = 0.2;
  ;
  a.color.a = 1;
  a.lifetime = ros::Duration(5.0);
  a.frame_locked = false;
  path_pub_.publish(a);
}

void Rrt::visualizePose(geometry_msgs::Pose pose, int id)
{
  visualization_msgs::Marker a;
  a.header.stamp = ros::Time::now();
  a.header.seq = id;
  a.header.frame_id = frame_id_;
  a.id = id;
  a.ns = "pose";
  a.type = visualization_msgs::Marker::ARROW;
  a.action = visualization_msgs::Marker::ADD;
  a.pose = pose;
  a.scale.x = 0.4;
  a.scale.y = 0.1;
  a.scale.z = 0.1;
  a.color.r = 1.0;
  a.color.g = 0.0;
  a.color.b = 0.0;
  a.color.a = 1.0;
  a.lifetime = ros::Duration(5.0);
  a.frame_locked = false;

  path_pub_.publish(a);
}

void Rrt::visualizeEdge(RrtNode* node, int id)
{
  visualization_msgs::Marker a;
  a.header.stamp = ros::Time::now();
  a.header.seq = id;
  a.header.frame_id = frame_id_;
  a.id = id;
  a.ns = "vp_branches";
  a.type = visualization_msgs::Marker::ARROW;
  a.action = visualization_msgs::Marker::ADD;
  a.pose.position.x = node->parent->pos[0];
  a.pose.position.y = node->parent->pos[1];
  a.pose.position.z = node->parent->pos[2];
  Eigen::Quaternion<double> q;
  Eigen::Vector3d init(1.0, 0.0, 0.0);
  Eigen::Vector3d dir(node->pos[0] - node->parent->pos[0],
                      node->pos[1] - node->parent->pos[1],
                      node->pos[2] - node->parent->pos[2]);
  q.setFromTwoVectors(init, dir);
  q.normalize();
  a.pose.orientation.x = q.x();
  a.pose.orientation.y = q.y();
  a.pose.orientation.z = q.z();
  a.pose.orientation.w = q.w();
  a.scale.x = dir.norm();
  a.scale.y = 0.05;
  a.scale.z = 0.05;
  a.color.r = 1;
  a.color.g = 0.3;
  a.color.b = 0.7;
  a.color.a = 1.0;
  a.lifetime = ros::Duration(5.0);
  a.frame_locked = false;

  path_pub_.publish(a);
}

void Rrt::visualizePath(RrtNode* node)
{
  for (int id = 0; node->parent; ++id)
  {
    visualization_msgs::Marker a;
    a.header.stamp = ros::Time::now();
    a.header.seq = id;
    a.header.frame_id = frame_id_;
    a.id = id;
    a.ns = "path";
    a.type = visualization_msgs::Marker::ARROW;
    a.action = visualization_msgs::Marker::ADD;
    a.pose.position.x = node->parent->pos[0];
    a.pose.position.y = node->parent->pos[1];
    a.pose.position.z = node->parent->pos[2];
    Eigen::Quaternion<double> q;
    Eigen::Vector3d init(1.0, 0.0, 0.0);
    Eigen::Vector3d dir(node->pos[0] - node->parent->pos[0],
                        node->pos[1] - node->parent->pos[1],
                        node->pos[2] - node->parent->pos[2]);
    q.setFromTwoVectors(init, dir);
    q.normalize();
    a.pose.orientation.x = q.x();
    a.pose.orientation.y = q.y();
    a.pose.orientation.z = q.z();
    a.pose.orientation.w = q.w();
    a.scale.x = dir.norm();
    a.scale.y = 0.07;
    a.scale.z = 0.07;
    a.color.r = 0.7;
    a.color.g = 0.7;
    a.color.b = 0.3;
    a.color.a = 1.0;
    a.lifetime = ros::Duration(100.0);
    a.frame_locked = false;

    path_pub_.publish(a);

    node = node->parent;
  }
}

void Rrt::visualizeGoals(std::vector<geometry_msgs::Pose> goals)
{
  for (int i = 0; i < goals.size(); ++i)
  {
    visualization_msgs::Marker a;
    a.header.stamp = ros::Time::now();
    a.header.seq = i;
    a.header.frame_id = frame_id_;
    a.id = i;
    a.ns = "goals";
    a.type = visualization_msgs::Marker::ARROW;
    a.action = visualization_msgs::Marker::ADD;
    a.pose = goals[i];

    a.scale.x = 0.2;
    a.scale.y = 0.1;
    a.scale.z = 0.1;
    a.color.r = 1.0;
    a.color.g = 0.3;
    a.color.b = 0.7;
    a.color.a = 1;
    a.lifetime = ros::Duration(100.0);
    a.frame_locked = false;
    path_pub_.publish(a);
  }
}

}  // namespace aeplanner_ns
