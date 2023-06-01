#include <ros/ros.h>

#include <aeplanner/param.h>

namespace aeplanner
{
  Params readParams()
  {
    // FIXME namespaces
    Params params;
    std::string ns = ros::this_node::getNamespace();
    params.hfov = 60;
    if (!ros::param::get(ns + "/camera/horizontal_fov", params.hfov)) {
      ROS_WARN_STREAM("No horizontal fov specified. Default: " << params.hfov);
    }
    params.vfov = 45;
    if (!ros::param::get(ns + "/camera/vertical_fov", params.vfov)) {
      ROS_WARN_STREAM("No vertical fov specified. Default: " << params.vfov);
    }
    params.dr = 0.2;
    if (!ros::param::get("/res", params.dr)) {
      ROS_WARN_STREAM("Could not read octomap resolution. Looking for /res");
      ROS_WARN_STREAM("Using resolution specified by param file instead");
    }
    else if (!ros::param::get(ns + "/raycast/dr", params.dr)) {
      ROS_WARN_STREAM("No dr specified. Default: " << params.dr);
    }
    params.dphi = 10;
    if (!ros::param::get(ns + "/raycast/dphi", params.dphi)) {
      ROS_WARN_STREAM("No dphi specified. Default: " << params.dphi);
    }
    params.dtheta = 10;
    if (!ros::param::get(ns + "/raycast/dtheta", params.dtheta)) {
      ROS_WARN_STREAM("No dtheta specified. Default: " << params.dtheta);
    }
    params.lambda = 0.5;
    if (!ros::param::get(ns + "/aep/gain/lambda", params.lambda)) {
      ROS_WARN_STREAM("No lambda specified. Default: " << params.lambda);
    }

    params.extension_range = 1.0;
    if (!ros::param::get(ns + "/aep/tree/extension_range", params.extension_range)) {
      ROS_WARN_STREAM("No extension_range specified. Default: " << params.extension_range);
    }
    params.max_sampling_radius = 10.0;
    if (!ros::param::get(ns + "/aep/tree/max_sampling_radius", params.max_sampling_radius)) {
      ROS_WARN_STREAM("No max_sampling_radius specified. Default: " << params.max_sampling_radius);
    }
    params.sigma_thresh = 0.2;
    if (!ros::param::get(ns + "/aep/gain/sigma_thresh", params.sigma_thresh)) {
      ROS_WARN_STREAM("No sigma_thresh specified. Default: " << params.sigma_thresh);
    }
    params.init_iterations = 15;
    if (!ros::param::get(ns + "/aep/tree/initial_iterations", params.init_iterations)) {
      ROS_WARN_STREAM("No init_iterations specified. Default: " << params.init_iterations);
    }
    params.r_max = 4.0;
    if (!ros::param::get(ns + "/aep/gain/r_max", params.r_max)) {
      ROS_WARN_STREAM("No r_max specified. Default: " << params.r_max);
    }
    params.r_min = 0.5;
    if (!ros::param::get(ns + "/aep/gain/r_min", params.r_min)) {
      ROS_WARN_STREAM("No r_min specified. Default: " << params.r_min);
    }
    if (!ros::param::get(ns + "/boundary/min", params.boundary_min)) {
      ROS_WARN_STREAM("No boundary/min specified.");
    }
    if (!ros::param::get(ns + "/boundary/max", params.boundary_max)) {
      ROS_WARN_STREAM("No boundary/max specified.");
    }
    params.bounding_radius;
    if (!ros::param::get(ns + "/system/bbx/r", params.bounding_radius)) {
      ROS_WARN_STREAM("No /system/bbx/r specified. Default: " << params.bounding_radius);
    }
    params.cutoff_iterations = 200;
    if (!ros::param::get(ns + "/aep/tree/cutoff_iterations", params.cutoff_iterations)) {
      ROS_WARN_STREAM("No /aep/tree/cutoff_iterations specified. Default: " << params.cutoff_iterations);
    }
    params.zero_gain = 0.0;
    if (!ros::param::get(ns + "/aep/gain/zero", params.zero_gain)) {
      ROS_WARN_STREAM("No /aep/gain/zero specified. Default: " << params.zero_gain);
    }
    params.d_overshoot_ = 0.5;
    if (!ros::param::get(ns + "/system/bbx/overshoot", params.d_overshoot_)) {
      ROS_WARN_STREAM("No /system/bbx/overshoot specified. Default: " << params.d_overshoot_);
    }
    params.world_frame = "world";
    if (!ros::param::get(ns + "/world_frame", params.world_frame)) {
      ROS_WARN_STREAM("No /world_frame specified. Default: " << params.world_frame);
    }
    params.robot_frame = "base_link";
    if (!ros::param::get(ns + "/robot_frame", params.robot_frame)) {
      ROS_WARN_STREAM("No /robot_frame specified. Default: " << params.robot_frame);
    }
    params.visualize_tree = false;
    if (!ros::param::get(ns + "/visualize_tree", params.visualize_tree)) {
      ROS_WARN_STREAM("No /visualize_tree specified. Default: " << params.visualize_tree);
    }
    params.visualize_rays = false;
    if (!ros::param::get(ns + "/visualize_rays", params.visualize_rays)) {
      ROS_WARN_STREAM("No /visualize_rays specified. Default: " << params.visualize_rays);
    }
    params.visualize_exploration_area = false;
    if (!ros::param::get(ns + "/visualize_exploration_area", params.visualize_exploration_area)) {
      ROS_WARN_STREAM("No /visualize_exploration_area specified. Default: " << params.visualize_exploration_area);
    }

    // daep
    params.max_sampled_initial_nodes = 30;
    if (!ros::param::get(ns + "/daep/rrt/max_sampled_initial_nodes", params.max_sampled_initial_nodes)) {
      ROS_WARN_STREAM("No /daep/rrt/max_sampled_initial_nodes specified. Default: " << params.max_sampled_initial_nodes);
    }

    params.max_sampled_nodes = 50;
    if (!ros::param::get(ns + "/daep/rrt/max_sampled_nodes", params.max_sampled_nodes)) {
      ROS_WARN_STREAM("No /daep/rrt/max_sampled_nodes specified. Default: " << params.max_sampled_nodes);
    }

    params.cache_node_threshold = 30;
    if (!ros::param::get(ns + "/daep/gain/cache_node_threshold", params.cache_node_threshold)) {
      ROS_WARN_STREAM("No cache_node_threshold specified. Default: " << params.cache_node_threshold);
    }
    params.node_gain_threshold = 10;
    if (!ros::param::get(ns + "/daep/gain/node_gain_threshold", params.node_gain_threshold)) {
      ROS_WARN_STREAM("No node_gain_threshold specified. Default: " << params.node_gain_threshold);
    }
    params.zeta = 0.5;
    if (!ros::param::get(ns + "/daep/dfm/zeta", params.zeta)) {
      ROS_WARN_STREAM("No zeta specified. Default: " << params.zeta);
    }
    params.KFiterations = 20;
    if (!ros::param::get(ns + "/daep/kf/iterations", params.KFiterations)) {
      ROS_WARN_STREAM("No KFiterations specified. Default: " << params.KFiterations);
    }
    params.time_step = 0.5;
    if (!ros::param::get(ns + "/daep/kf/time_step", params.time_step)) {
      ROS_WARN_STREAM("No time_step specified. Default: " << params.time_step);
    }

    params.visualize_static_and_dynamic_rays = true;
    if (!ros::param::get(ns + "/visualize_static_and_dynamic_rays", params.visualize_static_and_dynamic_rays)) {
      ROS_WARN_STREAM("No visualize_static_and_dynamic_rays specified. Default: " << params.visualize_static_and_dynamic_rays);
    }

    params.human_height = 1.8;
    if (!ros::param::get(ns + "/human_height", params.human_height)) {
      ROS_WARN_STREAM("No human_height specified. Default: " << params.human_height);
    }

    params.human_width = 1;
    if (!ros::param::get(ns + "/human_width", params.human_width)) {
      ROS_WARN_STREAM("No human_width specified. Default: " << params.human_width);
    }

    params.human_linear_velocity = 0.35;
    if (!ros::param::get(ns + "/human_linear_velocity", params.human_linear_velocity)) {
      ROS_WARN_STREAM("No human_linear_velocity specified. Default: " << params.human_linear_velocity);
    }

    params.human_angular_velocity = 1;
    if (!ros::param::get(ns + "/human_angular_velocity", params.human_angular_velocity)) {
      ROS_WARN_STREAM("No human_angular_velocity specified. Default: " << params.human_angular_velocity);
    }
    
    params.drone_height = 0.4;
    if (!ros::param::get(ns + "/drone_height", params.drone_height)) {
      ROS_WARN_STREAM("No drone_height specified. Default: " << params.drone_height);
    }

    params.drone_width = 0.1;
    if (!ros::param::get(ns + "/drone_width", params.drone_width)) {
      ROS_WARN_STREAM("No drone_width specified. Default: " << params.drone_width);
    }

    params.drone_linear_velocity = 0.35;
    if (!ros::param::get(ns + "/drone_linear_velocity", params.drone_linear_velocity)) {
      ROS_WARN_STREAM("No drone_linear_velocity specified. Default: " << params.drone_linear_velocity);
    }

    params.drone_angular_velocity = 1;
    if (!ros::param::get(ns + "/drone_angular_velocity", params.drone_angular_velocity)) {
      ROS_WARN_STREAM("No drone_angular_velocity specified. Default: " << params.drone_angular_velocity);
    }

    params.boosted_boundary_length = 1;
    if (!ros::param::get(ns + "/boosted_boundary_length", params.boosted_boundary_length)) {
          ROS_WARN_STREAM("No boosted_boundary_length specified. Default: " << params.boosted_boundary_length);
    }

    params.boost_magnitude = 5;
    if (!ros::param::get(ns + "/boost_magnitude", params.boost_magnitude)) {
          ROS_WARN_STREAM("No boost_magnitude specified. Default: " << params.boost_magnitude);
    }

    params.drone_freeze = false;
    if (!ros::param::get(ns + "/drone_freeze", params.drone_freeze)) {
          ROS_WARN_STREAM("No drone_freeze specified. Default: " << params.drone_freeze);
    }

    params.global_planner_counter = 25;
    if (!ros::param::get(ns + "/global_planner_counter", params.global_planner_counter)) {
        ROS_WARN_STREAM("No global_planner_counter specified. Default: " << params.global_planner_counter);
    }

    params.look_ahead_horizon = 5;
    if (!ros::param::get(ns + "/look_ahead_horizon", params.look_ahead_horizon)) {
        ROS_WARN_STREAM("No look_ahead_horizon specified. Default: " << params.look_ahead_horizon);
    }

    return params;
  }
}
