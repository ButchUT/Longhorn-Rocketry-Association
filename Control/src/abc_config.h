#ifndef CONTROL_ABC_CONFIG_H
#define CONTROL_ABC_CONFIG_H

struct AirbrakeControllerConfiguration {
  // Controller
  float target_altitude;
  int bounds_history_size, regression_id;
  bool enforce_bounds_history_size;

  // Brake step profiler
  float bs_profile_step_min, bs_profile_step_max;
  float bs_profile_velocity_min, bs_profile_velocity_max;
  float bs_profile_exp;

  // Brake step controller
  int bsc_history_size;
  float bsc_thresh_osc, bsc_thresh_stb;

  float bsc_down_profile_weight_min, bsc_down_profile_weight_max;
  float bsc_down_profile_velocity_min, bsc_down_profile_velocity_max;
  float bsc_down_profile_exp;
  float bsc_up_profile_weight_min, bsc_up_profile_weight_max;
  float bsc_up_profile_velocity_min, bsc_up_profile_velocity_max;
  float bsc_up_profile_exp;
};

#endif
