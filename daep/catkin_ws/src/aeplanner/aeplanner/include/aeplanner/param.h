#ifndef READ_PARAMS_H
#define READ_PARAMS_H

namespace aeplanner
{
  struct Params
  {
    double hfov;
    double vfov;
    double r_max;
    double r_min;

    double dr;
    double dphi;
    double dtheta;

    double lambda;
    double zero_gain;
    double extension_range;
    double max_sampling_radius;
    double sigma_thresh;
    double zeta;
    double time_step;
    
    double d_overshoot_;
    double bounding_radius;

    int init_iterations;
    int cutoff_iterations;
    int cache_node_threshold;
    int node_gain_threshold;
    int KFiterations;
    
    std::vector<double> boundary_min;
    std::vector<double> boundary_max;

    std::string robot_frame;
    std::string world_frame;

    bool visualize_tree;
    bool visualize_rays;
    bool visualize_exploration_area;
    bool visualize_static_and_dynamic_rays;

    double human_height;
    double human_width;
    double human_linear_velocity;
    double human_angular_velocity;

    double drone_height;
    double drone_width;
    double drone_linear_velocity;
    double drone_angular_velocity;

    double boosted_boundary_length;
    double boost_magnitude;

    double max_sampled_initial_nodes;
    double max_sampled_nodes;

    bool drone_freeze;

    double global_planner_counter;

    int look_ahead_horizon;

  };

  Params readParams();
}

#endif
