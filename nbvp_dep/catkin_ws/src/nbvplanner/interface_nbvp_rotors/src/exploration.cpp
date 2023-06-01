/*
 * Copyright 2015 Andreas Bircher, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <thread>
#include <chrono>

#include <ros/ros.h>
#include <ros/package.h>
#include <tf/tf.h>
#include <std_srvs/Empty.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <nbvplanner/nbvp_srv.h>
#include "interface_nbvp_rotors/Goal.h"
#include <std_msgs/Bool.h>

// Own
#include <fstream>
#include <istream>
#include <aeplanner_evaluation/Coverage.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <cstdlib> //std::getenv
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

int const GOAL_PUB_FREQ = 15;
bool goal_reached = false;
bool prev_goal_reached = false;
double path_length = 0.0;
std_msgs::Bool exploration_completed_msg;
ros::Publisher exploration_completion_pub;


//Avoidance
bool prev_avoidance = false;
bool avoidance = false;
geometry_msgs::PoseStamped new_goal;
geometry_msgs::PoseStamped prev_goal;
geometry_msgs::Pose current_pose;

void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    current_pose = msg->pose.pose;
    exploration_completion_pub.publish(exploration_completed_msg);
}

// Calculate distance between two poses and add to the normal path length
void add_distance(geometry_msgs::Pose pose1, geometry_msgs::Pose pose2) {
  double dx = pose1.position.x - pose2.position.x;
  double dy = pose1.position.y - pose2.position.y;
  double dz = pose1.position.z - pose2.position.z;
  double distance = sqrt((dx*dx) + (dy*dy) + (dz*dz));
  path_length += distance;
}

void goalReachedCallback(const std_msgs::Bool::ConstPtr& msg)
{
    //ROS_INFO("Received goal reached message: %d", msg->data);
    // Lock 'goal_reached' so that we don't send consecutive 'True' and miss a fly
    if (msg->data != prev_goal_reached) {
        goal_reached = msg->data;
        prev_goal_reached = goal_reached;
    } else {
      prev_goal_reached = false;
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

void publishGoalAndWait(ros::Publisher goal_pub, int const& freq, interface_nbvp_rotors::Goal const& goal){
  /*
    Publishes goal continously and waits for motion planner /goal_reached topic.
  */

  geometry_msgs::Pose start_pose = current_pose;
  geometry_msgs::Pose last_pose = current_pose;

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
          add_distance(last_pose, current_pose);
          add_distance(current_pose, new_goal.pose);
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

  exploration_completed_msg.data = false;

  ROS_INFO("Started exploration"); 

  std_srvs::Empty srv;
  bool unpaused = ros::service::call("/gazebo/unpause_physics", srv);
  unsigned int i = 0;

  ros::Publisher goal_pub = nh.advertise<interface_nbvp_rotors::Goal>("goal", 0);
  ros::Subscriber gr_sub = nh.subscribe("/goal_reached", 1, goalReachedCallback);
  geometry_msgs::PoseWithCovarianceStamped::ConstPtr init_pose = ros::topic::waitForMessage<geometry_msgs::PoseWithCovarianceStamped>("/pose");
  current_pose = init_pose->pose.pose;
  geometry_msgs::Pose last_pose = init_pose->pose.pose;


  // Avoidance mode
  ros::Subscriber avoid_sub = nh.subscribe("/avoidance", 1, avoidanceCallback);
  ros::Subscriber new_goal = nh.subscribe("/new_goal", 1, newGoalCallback);

  // Completion
  exploration_completion_pub = nh.advertise<std_msgs::Bool>("/exploration_completed", 1);
  // pose subscriber
  ros::Subscriber sub_pose = nh.subscribe("/pose", 10, poseCallback);

  // Evaluation
  std::string homePath = std::getenv("HOME");  
  std::ofstream logfile, pathfile, coveragefile;
  logfile.open(homePath + "/data/logfile.csv");
  pathfile.open(homePath + "/data/path.csv");
  coveragefile.open(homePath + "/data/coverage.csv");
  // File Headers
  logfile << "Iteration, " << "Path length, " << "Time, " << "Planning, " << "Flying, " << std::endl;
  pathfile << "Goal x, " << "Goal y, " << "Goal z, " << std::endl;
  coveragefile << "Time, " << "Coverage (m3), " << "Coverage (%), " << "Free space, " << "Occupied Space, " << "Unmapped Space" << std::endl;
  
  // Collision
  ros::Publisher collision_pub = nh.advertise<std_msgs::Bool>("/write_log", 1);
  ros::Publisher clock_start_pub = nh.advertise<std_msgs::Bool>("/clock_start", 1);

  // Coverage
  ros::ServiceClient coverage_srv = nh.serviceClient<aeplanner_evaluation::Coverage>("/get_coverage");
  // Create a request message
  aeplanner_evaluation::CoverageRequest srv_request;
  // Call the service and store the response in 'res'
  aeplanner_evaluation::CoverageResponse srv_response;


  // Trying to unpause Gazebo for 10 seconds.
  while (i <= 10 && !unpaused) {
    ROS_INFO("Wait for 1 second before trying to unpause Gazebo again.");
    std::this_thread::sleep_for(std::chrono::seconds(1));
    unpaused = ros::service::call("/gazebo/unpause_physics", srv);
    ++i;
  }

  if (!unpaused) {
    ROS_FATAL("Could not wake up Gazebo.");
    return -1;
  } else {
    ROS_INFO("Unpaused the Gazebo simulation.");
  }

  double dt = 1.0;
  std::string ns = ros::this_node::getName();
  if (!ros::param::get(ns + "/nbvp/dt", dt)) {
    ROS_FATAL("Could not start exploration. Parameter missing! Looking for %s",
              (ns + "/nbvp/dt").c_str());
    return -1;
  }

  // Motion Planner Goal
  interface_nbvp_rotors::Goal goal;

  // Wait for 5 seconds to let the Gazebo GUI show up.
  ros::Duration(5.0).sleep();
  ros::Duration(5.0).sleep();
  // Initial motion
  ROS_INFO("Performing initial motion"); 
  goal.x = init_pose->pose.pose.position.x;
  goal.y = init_pose->pose.pose.position.y;
  goal.z = init_pose->pose.pose.position.z;
  goal.yaw = 2;
  publishGoalAndWait(goal_pub, GOAL_PUB_FREQ, goal);
  goal.yaw = 4;
  publishGoalAndWait(goal_pub, GOAL_PUB_FREQ, goal);
  goal.yaw = 0;
  publishGoalAndWait(goal_pub, GOAL_PUB_FREQ, goal);

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
  logfile << 0 << ", " << 0 << ", " << 0 << ", " << 0 << ", " << 0 << std::endl;

  // Start planning: The planner is called and the computed path sent to the controller.
  int iteration = 1;

 

  //Start the Collision clock
  std_msgs::Bool planning_started;
  planning_started.data = true;
  clock_start_pub.publish(planning_started);

  // Logging
  ros::Duration planning_time;
  double total_fly_time = 0;
  double total_planning_time = 0;
  ros::Duration elapsed;
  ros::Duration fly_time;
  int not_reachable = 0;

  ros::Time start = ros::Time::now();
  while (ros::ok()) 
  {
    ROS_INFO_THROTTLE(0.5, "Planning iteration %i", iteration);
    nbvplanner::nbvp_srv planSrv;
    planSrv.request.header.stamp = ros::Time::now();
    planSrv.request.header.seq = iteration;
    planSrv.request.header.frame_id = "world";

    ros::Time planning_start = ros::Time::now();
    
    if (ros::service::call("nbvplanner", planSrv)) 
    {
      
      planning_time = ros::Time::now() - planning_start;
      
      if (planSrv.response.path.size() == 0) 
      {
        ros::Duration(1.0).sleep();
      }

      ros::Time fly_start = ros::Time::now();
      for (int i = 0; i < planSrv.response.path.size(); i++) 
      {
        // Only plan to every sixth waypoint and always include the last waypoint
        if (i % 8 == 0 || i == planSrv.response.path.size()-1)
        {
          tf::Pose pose_dep;
          tf::poseMsgToTF(planSrv.response.path[i], pose_dep);
          double yaw = tf::getYaw(pose_dep.getRotation());

          if(yaw < 0) yaw += 2*M_PI;

          goal.x = planSrv.response.path[i].position.x;
          goal.y = planSrv.response.path[i].position.y;
          goal.z = planSrv.response.path[i].position.z;
          goal.yaw = yaw;
          
          // Log positions
          pathfile << goal.x << ", " << goal.y << ", " << goal.z << std::endl;
          
          // Distance
          geometry_msgs::Pose tmp_pose;
          tmp_pose.position.x = goal.x; // Set the x coordinate
          tmp_pose.position.y = goal.y; // Set the y coordinate
          tmp_pose.position.z = goal.z; // Set the z coordinate

          // Publish to motion planner
          publishGoalAndWait(goal_pub, GOAL_PUB_FREQ, goal);
          last_pose = tmp_pose;
        
          ros::Duration(dt).sleep();
        }
      }
      fly_time = ros::Time::now() - fly_start;
    } 
    else 
    {
      ROS_WARN_THROTTLE(1, "Planner not reachable");
      ros::Duration(1.0).sleep();
      
      if(not_reachable > 3)
      {
        break;
      }
      not_reachable++;
    }

    elapsed = ros::Time::now() - start;
    total_fly_time += fly_time.toSec();
    total_planning_time += planning_time.toSec();

    ROS_INFO_STREAM("Iteration: "       << iteration << "  " <<
                    "Path Length: "     << path_length << "  " <<
                    "Time: "            << elapsed << "  " <<
                    "Planning: "        << total_planning_time << "  " <<
                    "Flying: "          << total_fly_time);
    
    
    logfile << iteration << ", " 
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
        coveragefile << elapsed << ", " <<coverage_m3 << ", " <<  coverage_p << ", " << free_space << ", "  << occupied_space << ", " <<  unmapped_space << std::endl;
    } else {
        ROS_ERROR("Failed to call service /get_coverage");
    }
    // Collision
    std_msgs::Bool msg;
    msg.data = true;
    collision_pub.publish(msg);

    iteration++;
  }

logfile << iteration-1 << ", " 
        << path_length << ", "
        << elapsed << ", "
        << total_planning_time << ", "
        << total_fly_time << std::endl;
  
  std_msgs::Bool msg;
  msg.data = true;
  collision_pub.publish(msg);

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

  // completion
  exploration_completed_msg.data = true;
  exploration_completion_pub.publish(exploration_completed_msg);
  
  pathfile.close();
  logfile.close();
  coveragefile.close();
}
