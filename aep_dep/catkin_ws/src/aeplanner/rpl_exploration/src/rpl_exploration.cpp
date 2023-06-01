#include <fstream>
#include <ros/package.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <aeplanner_evaluation/Coverage.h>
#include <std_srvs/Empty.h>
#include <actionlib/client/simple_action_client.h>
#include <aeplanner/Node.h>
#include <aeplanner/aeplannerAction.h>
#include <rpl_exploration/FlyToAction.h>
#include <rrtplanner/rrtAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include "rpl_exploration/Goal.h"
#include <nav_msgs/Path.h>
#include <tf2/utils.h>
#include <std_msgs/Bool.h>
#include <cstdlib>
#include <nav_msgs/Odometry.h>

// Integration of motion planner
bool prev_goal_reached = false;
bool goal_reached = false;
int const GOAL_PUB_FREQ = 15;
double path_length = 0.0;
ros::Publisher exploration_completion_pub;
std_msgs::Bool exploration_completed_msg;

//Avoidance
bool prev_avoidance = false;
bool avoidance = false;
geometry_msgs::PoseStamped new_goal;
geometry_msgs::PoseStamped prev_goal;
geometry_msgs::PoseStamped current_pose_stamped;

// Calculate distance between two poses and add to the dynamic path length
void add_distance(geometry_msgs::Pose pose1, geometry_msgs::Pose pose2) {
  double dx = pose1.position.x - pose2.position.x;
  double dy = pose1.position.y - pose2.position.y;
  double dz = pose1.position.z - pose2.position.z;
  double distance = sqrt((dx*dx) + (dy*dy) + (dz*dz));
  path_length += distance;
}


void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  current_pose_stamped = *msg;
  exploration_completion_pub.publish(exploration_completed_msg);
}

void goalReachedCallback(const std_msgs::Bool::ConstPtr& msg)
{
    // Lock 'goal_reached' so that we don't send consecutive 'True' and miss a fly
    if (msg->data != prev_goal_reached)
    {
        goal_reached = msg->data;
        prev_goal_reached = goal_reached;
    }
}

void avoidanceCallback(const std_msgs::Bool::ConstPtr& msg)
{
    if (msg->data != prev_avoidance)
    { 
        avoidance = msg->data;
        prev_avoidance = avoidance;
    }
}

void newGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{    
    if(msg->pose != prev_goal.pose)
    {
      prev_goal = new_goal;
      new_goal = *msg;
    }
}

void publishGoalAndWait(ros::Publisher goal_pub, int const& freq, rpl_exploration::Goal const& goal){
  /*
    Publishes goal continously and waits for motion planner /goal_reached topic.
  */

  geometry_msgs::Pose start_pose = current_pose_stamped.pose;

  geometry_msgs::Pose last_pose = current_pose_stamped.pose;

  ros::Rate rate(freq); 

  while (ros::ok())
    {
      // Publish the goal pose
      goal_pub.publish(goal);
      ros::spinOnce();

      if(avoidance)
      {
        //Avoidance mode is active, a new goal has been published
        if(new_goal.pose != prev_goal.pose)
        {
          //If the new goal differs from the previous one, we add it to the path length
          //This to make sure we add each new goal only once
          add_distance(last_pose, current_pose_stamped.pose);
          add_distance(current_pose_stamped.pose, new_goal.pose);
          prev_goal = new_goal;
          last_pose = new_goal.pose;
        }
      }

        // Check if the goal has been reached
      if (goal_reached)
      {
            geometry_msgs::Pose final_pose;
            final_pose.position.x = goal.x;
            final_pose.position.y = goal.y;
            final_pose.position.z = goal.z;

            if(start_pose == last_pose)
            {
                //No avoidance was needed, add the straight path
                //from start pose to final pose
                add_distance(start_pose, final_pose);
            }
            else
            {
                //Avoidance was needed, and the final distance traveled
                //from the latest pose to the final pose
                add_distance(last_pose, final_pose);          
            }
            
            // Stop publishing the goal pose
            break;
      }
        rate.sleep();    
    }
  goal_reached = false;
}


int main(int argc, char** argv)
{
  
  ros::init(argc, argv, "exploration");
  ros::NodeHandle nh;
  ROS_INFO("Started exploration");
  exploration_completed_msg.data = false;

  // Open logfile;
  std::string path = ros::package::getPath("rpl_exploration");
  std::ofstream logfile, pathfile, coveragefile;
  // Get the home directory path
  std::string homePath = std::getenv("HOME");
  logfile.open(homePath + "/data/logfile.csv");
  pathfile.open(homePath + "/data/path.csv");
  coveragefile.open(homePath + "/data/coverage.csv");
  // File Headers
  logfile << "Iteration, " << "Path length, " << "Time, " << "Planning, " << "Flying, " << std::endl;
  pathfile << "Goal x, " << "Goal y, " << "Goal z, " << "Planner" << std::endl;
  coveragefile << "Time, " << "Coverage (m3), " << "Coverage (%), " << "Free space, " << "Occupied Space, " << "Unmapped Space" << std::endl;
  
  // DEP
  rpl_exploration::Goal goal;
  ros::Publisher goal_pub = nh.advertise<rpl_exploration::Goal>("goal", 0);
  ros::Subscriber gr_sub = nh.subscribe("/goal_reached", 1, goalReachedCallback);
  ros::Publisher collision_pub = nh.advertise<std_msgs::Bool>("/write_log", 1);
  ros::Publisher clock_start_pub = nh.advertise<std_msgs::Bool>("/clock_start", 1);

  // Avoidance mode
  ros::Subscriber avoid_sub = nh.subscribe("/avoidance", 1, avoidanceCallback);
  ros::Subscriber new_goal = nh.subscribe("/new_goal", 1, newGoalCallback);

  // Coverage service
  ros::ServiceClient coverage_srv = nh.serviceClient<aeplanner_evaluation::Coverage>("/aeplanner/get_coverage");

  // Create a request message
  aeplanner_evaluation::CoverageRequest srv_request;
  // Call the service and store the response in 'res'
  aeplanner_evaluation::CoverageResponse srv_response;

  // pose subscriber
  ros::Subscriber sub_pose = nh.subscribe("/pose", 10, poseCallback);

  // wait for aep server to start
  ROS_INFO("Waiting for aeplanner action server");
  actionlib::SimpleActionClient<aeplanner::aeplannerAction> aep_ac("make_plan", true);
  aep_ac.waitForServer();  // will wait for infinite time
  ROS_INFO("aeplanner action server started!");

  ROS_INFO("Waiting for rrt action server");
  actionlib::SimpleActionClient<rrtplanner::rrtAction> rrt_ac("rrt", true);
  rrt_ac.waitForServer();
  ROS_INFO("rrt Action server started!");

  // Get current pose
  geometry_msgs::PoseStamped::ConstPtr current_pose;
  geometry_msgs::PoseStamped::ConstPtr init_pose = ros::topic::waitForMessage<geometry_msgs::PoseStamped>("/pose");
  current_pose_stamped = *init_pose;

  geometry_msgs::Pose last_pose;
  last_pose = init_pose->pose;
  ROS_INFO("init position: x=%f, y=%f, z=%f, yaw=%f", init_pose->pose.position.x, init_pose->pose.position.y, init_pose->pose.position.z, tf2::getYaw(init_pose->pose.orientation));
  
  // collision
  std_msgs::Bool msg;
  msg.data = true;

  // completion
  exploration_completion_pub = nh.advertise<std_msgs::Bool>("/exploration_completed", 1);
  if (coverage_srv.call(srv_request, srv_response)) {
        // save the coverage, free space, occupied space, and unmapped space in variables in your program
        // Extract the coverage data from the response
        double coverage_m3 = srv_response.coverage_m3;
        double coverage_p = srv_response.coverage_p;
        double free_space = srv_response.free;
        double occupied_space = srv_response.occupied;
        double unmapped_space = srv_response.unmapped;
        coveragefile << 0 << ", " << 0 << ", " <<  0 << ", " << 0 << ", "  << 0 << ", " <<  unmapped_space+occupied_space+free_space << std::endl;
    } else {
        ROS_ERROR("Failed to call service /get_coverage");
    }
  logfile << 0 << ", " 
            << 0 << ", "
            << 0 << ", "
            << 0 << ", "
            << 0 << std::endl;

  // Perfoming initial motion (warmup)
  ROS_INFO("Performing initial motion");
  goal.x = init_pose->pose.position.x;
  goal.y = init_pose->pose.position.y;
  goal.z = init_pose->pose.position.z;
  goal.yaw = 2;
  publishGoalAndWait(goal_pub, GOAL_PUB_FREQ, goal);
  goal.yaw = 4;
  publishGoalAndWait(goal_pub, GOAL_PUB_FREQ, goal);
  goal.yaw = tf2::getYaw(init_pose->pose.orientation);
  publishGoalAndWait(goal_pub, GOAL_PUB_FREQ, goal);

  // Start planning: The planner is called and the computed path sent to the
  // controller.
  int iteration = 1;
  int actions_taken = 1;

  //Create time variables and start the timer
  ros::Time start = ros::Time::now();
  ros::Duration elapsed;
  ros::Duration planning_time;
  double total_fly_time = 0;
  double total_planning_time = 0;

  //Start the Collision clock
  std_msgs::Bool planning_started;
  planning_started.data = true;
  clock_start_pub.publish(planning_started);
  
  while (ros::ok())
  {
    ROS_INFO_STREAM("Planning iteration " << iteration);
    aeplanner::aeplannerGoal aep_goal;
    aep_goal.header.stamp = ros::Time::now();
    aep_goal.header.seq = iteration;
    aep_goal.header.frame_id = "world";
    aep_goal.actions_taken = actions_taken;
    ros::Time planning_start = ros::Time::now();
    aep_ac.sendGoal(aep_goal);

    while (!aep_ac.waitForResult(ros::Duration(0.05))){}

    ros::Duration fly_time;

    if (aep_ac.getResult()->is_clear){   
      planning_time = ros::Time::now() - planning_start;      

      actions_taken = 0;
      geometry_msgs::PoseStamped goal_pose = aep_ac.getResult()->pose;
      goal.x = goal_pose.pose.position.x;
      goal.y = goal_pose.pose.position.y;
      goal.z = goal_pose.pose.position.z;
      double yaw = tf2::getYaw(goal_pose.pose.orientation);
      if (yaw < 0) yaw += 2 * M_PI;
      goal.yaw = yaw;
      last_pose = goal_pose.pose;
      // Write path to file
      pathfile << goal_pose.pose.position.x << ", " << goal_pose.pose.position.y
               << ", " << goal_pose.pose.position.z << ", aep" << std::endl;
      
      ros::Time fly_start = ros::Time::now();

      // Continuously publish the goal pose until goal is reached
      publishGoalAndWait(goal_pub, GOAL_PUB_FREQ, goal);
      
      fly_time = ros::Time::now() - fly_start;
    }
    else
    { 
      rrtplanner::rrtGoal rrt_goal;
      rrt_goal.start.header.stamp = ros::Time::now();
      rrt_goal.start.header.frame_id = "world";
      rrt_goal.start.pose = last_pose;


      if (!aep_ac.getResult()->frontiers.poses.size())
      {
        ROS_WARN("Exploration complete!");
        //Stop the path length timer for the last time (get it? =D )
        break;
      }
      
      for (auto it = aep_ac.getResult()->frontiers.poses.begin();
           it != aep_ac.getResult()->frontiers.poses.end(); ++it)
      {
        rrt_goal.goal_poses.poses.push_back(*it);
      }

      rrt_ac.sendGoal(rrt_goal);

      while (!rrt_ac.waitForResult(ros::Duration(0.05))){}
      nav_msgs::Path path = rrt_ac.getResult()->path;
      
      planning_time = ros::Time::now() - planning_start;
      ros::Time fly_start = ros::Time::now();

      for (int i = path.poses.size() - 1; i >= 0; --i)
      {
        geometry_msgs::Pose goal_pose = path.poses[i].pose;
        last_pose = goal_pose;

        // Write path to file
        pathfile << goal_pose.position.x << ", " << goal_pose.position.y << ", "
                 << goal_pose.position.z << ", rrt" << std::endl;

        goal.x = goal_pose.position.x;
        goal.y = goal_pose.position.y;
        goal.z = goal_pose.position.z;
        double yaw = tf2::getYaw(goal_pose.orientation);
        if (yaw < 0) yaw += 2 * M_PI;
        goal.yaw = yaw;

        // Continuously publish the goal pose until goal is reached
        publishGoalAndWait(goal_pub, GOAL_PUB_FREQ, goal);
      }
      actions_taken = -1;
      fly_time = ros::Time::now() - fly_start;
    }

    elapsed = ros::Time::now() - start;
    total_fly_time += fly_time.toSec();
    total_planning_time += planning_time.toSec();
    
    logfile << iteration << ", " 
            << path_length << ", "    
            << elapsed << ", "
            << total_planning_time << ", "
            << total_fly_time << std::endl;


    ROS_INFO_STREAM("Iteration: "       << iteration << "  " <<
                    "Path Length: "       << path_length << "  " <<
                    "Time: "            << elapsed << "  " <<
                    "Planning: "        << total_planning_time << "  " <<
                    "Flying: "          << total_fly_time);


    iteration++;
    collision_pub.publish(msg);

  
    if (coverage_srv.call(srv_request, srv_response)) {
        // save the coverage, free space, occupied space, and unmapped space in variables in your program
        // Extract the coverage data from the response
        double coverage_m3 = srv_response.coverage_m3;
        double coverage_p = srv_response.coverage_p;
        double free_space = srv_response.free;
        double occupied_space = srv_response.occupied;
        double unmapped_space = srv_response.unmapped;
        coveragefile << elapsed << ", " <<coverage_m3 << ", " <<  coverage_p << ", " << free_space << ", "  << occupied_space << ", " <<  unmapped_space << std::endl;
    } else {
        ROS_ERROR("Failed to call service /get_coverage");
    }
    
  }
  
  //Extract the current time and calculate the distance flown
  // and add it to the extra column
  logfile << iteration-1 << ", " 
            << path_length << ", "
            << elapsed << ", "
            << total_planning_time << ", "
            << total_fly_time << std::endl;
  
  if (coverage_srv.call(srv_request, srv_response)) {
        // save the coverage, free space, occupied space, and unmapped space in variables in your program
        // Extract the coverage data from the response
        double coverage_m3 = srv_response.coverage_m3;
        double coverage_p = srv_response.coverage_p;
        double free_space = srv_response.free;
        double occupied_space = srv_response.occupied;
        double unmapped_space = srv_response.unmapped;
        coveragefile << elapsed << ", " << coverage_m3 << ", " <<  coverage_p << ", " << free_space << ", "  << occupied_space << ", " <<  unmapped_space << std::endl;
    } else {
        ROS_ERROR("Failed to call service /get_coverage");
  }
  // Completion
  exploration_completed_msg.data = true;
  exploration_completion_pub.publish(exploration_completed_msg);

  collision_pub.publish(msg);
  pathfile.close();
  logfile.close();
  coveragefile.close();
}
