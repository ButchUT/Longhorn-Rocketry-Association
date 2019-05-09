#include "abc_config.h"
#include "abc_preconfig.h"
#include "control_profiling.h"

AirbrakeController* make_2019_sac_abc() {
  AirbrakeControllerConfiguration config;
  config.target_altitude = 4600;
  config.bounds_history_size = 50;
  config.enforce_bounds_history_size = false;
  config.regression_id = abc::REG_NONE;

  config.bs_profile_velocity_min = 150;
  config.bs_profile_velocity_max = 343;
  config.bs_profile_step_min = 0.1;
  config.bs_profile_step_max = 0.075;
  config.bs_profile_exp = -1;

  config.bsc_history_size = 10;
  config.bsc_thresh_osc = 0.5;
  config.bsc_thresh_stb = 0.1;

  config.bsc_down_profile_velocity_min = config.bs_profile_velocity_min;
  config.bsc_down_profile_velocity_max = config.bs_profile_velocity_max;
  config.bsc_down_profile_weight_min = 0.5;
  config.bsc_down_profile_weight_max = 0.7;
  config.bsc_down_profile_exp = -1;

  config.bsc_up_profile_velocity_min = config.bs_profile_velocity_min;
  config.bsc_up_profile_velocity_max = config.bs_profile_velocity_max;
  config.bsc_up_profile_weight_min = 1.3;
  config.bsc_up_profile_weight_max = 1.5;
  config.bsc_up_profile_exp = -1;

  AirbrakeController *abc = new AirbrakeController(config);

  return abc;
}
