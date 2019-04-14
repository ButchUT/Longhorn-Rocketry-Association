#include "abc_preconfig.h"
#include "abc_profiling.h"

AirbrakeController* make_2019_sac_abc() {
  struct abc::AirbrakeControllerConfiguration config;
  config.target_altitude = 15000;
  config.min_velocity = 400;
  config.max_velocity = 1125.33 * 0.86;
  config.min_brake_step = 0.075;
  config.max_brake_step = 0.1;
  config.brake_step_profile_exp = -1;
  config.bounds_history_size = 50;
  config.enforce_bounds_history_size = false;
  config.use_polyreg = true;

  AirbrakeController *abc = new AirbrakeController(config);

  return abc;
}
