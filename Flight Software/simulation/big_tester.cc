#include "abc_preconfig.h"
#include <chrono>
#include "flight_sim.h"
#include <fstream>
#include <iostream>
#include "mock_sac_rocket.h"
#include "stats.h"
#include <string>
#include "telemetry.h"
#include "verlet_integrator.h"

double time() {
    chrono::milliseconds ms = chrono::duration_cast<chrono::milliseconds>(
        chrono::system_clock::now().time_since_epoch());
    return ms.count() / 1000.0;
}

bool compare_configs(Dataset &a, Dataset &b) {
  float MEAN_WEIGHT = 0.7;
  float STDEV_WEIGHT = 0.3;

  float a_score = fabs(a.mean()) * MEAN_WEIGHT + a.stdev() * STDEV_WEIGHT;
  float b_score = fabs(b.mean()) * MEAN_WEIGHT + b.stdev() * STDEV_WEIGHT;

  return a_score < b_score;
}

int main() {
  // Airbrake controller configurations
  const int NUM_ABC_CONFIGS = 3;

  struct abc::AirbrakeControllerConfiguration abc_config0;
  abc_config0.min_velocity = 400;
  abc_config0.max_velocity = 1125.33 * 0.86;
  abc_config0.min_brake_step = 0.075;
  abc_config0.max_brake_step = 0.1;
  abc_config0.brake_step_profile_exp = -1;
  abc_config0.bounds_history_size = 50;
  abc_config0.enforce_bounds_history_size = true;
  abc_config0.regression_id = abc::REG_EXP;

  struct abc::AirbrakeControllerConfiguration abc_config1 = abc_config0;
  abc_config1.enforce_bounds_history_size = false;

  struct abc::AirbrakeControllerConfiguration abc_config2 = abc_config1;
  abc_config2.regression_id = abc::REG_NONE;

  struct abc::AirbrakeControllerConfiguration abc_configs[] =
      {abc_config0, abc_config1, abc_config2};

  // Altitude targets
  const int NUM_ALTITUDE_TARGETS = 5;
  int target_altitudes[] = {15000, 15100, 15200, 15300, 15400};

  // Vehicle and simulator setup
  struct RocketData rocket_data;
  rocket_data.initial_altitude = 0;
  rocket_data.drag_coeff = 0.1;
  rocket_data.radius = 0.1;
  rocket_data.airbrake_area = 0.5;
  rocket_data.burnout_mass = 45;
  rocket_data.burnout_velocity = 1125.33 * 0.86;
  rocket_data.burnout_altitude = 1500;
  FlightSimulator flightsim = FlightSimulator(rocket_data);
  flightsim.set_stop_condition(flightsim::STOP_CONDITION_APOGEE);

  const int TRIALS_PER_FLIGHT = 10;
  int best_config = -1;
  Dataset best_config_set;

  // Run sims
  for (int i = 0; i < NUM_ABC_CONFIGS; i++) {
    Dataset config_altitude_error_set;

    for (int j = 0; j < NUM_ALTITUDE_TARGETS; j++) {
      for (int k = 0; k < TRIALS_PER_FLIGHT; k++) {
        abc::AirbrakeControllerConfiguration abc_config = abc_configs[i];
        abc_config.target_altitude = target_altitudes[j];

        MockSacRocket rocket = MockSacRocket(&flightsim, rocket_data,
            new AirbrakeController(abc_config));
        rocket.initialize();
        rocket.set_simulator(&flightsim);
        flightsim.begin(0);

        while (flightsim.is_running()) {
          flightsim.advance(0.1);
          rocket.set_timestep(flightsim.get_time());
          rocket.loop();
        }

        rocket.stop();
        config_altitude_error_set.add(abc_config.target_altitude -
            flightsim.get_rocket_altitude());
      }
    }

    std::cout << "ERROR SUMMARY - CONFIG " << i << std::endl <<
        "------------------------" << std::endl <<
        "mean=" << config_altitude_error_set.mean() << std::endl <<
        "var=" << config_altitude_error_set.var() << std::endl <<
        "stdev=" << config_altitude_error_set.stdev() << std::endl;

    if (best_config == -1 || compare_configs(config_altitude_error_set,
      best_config_set)) {
      best_config = i;
      best_config_set = config_altitude_error_set;
    }
  }

  std::cout << "Best config: " << best_config << std::endl;
}
