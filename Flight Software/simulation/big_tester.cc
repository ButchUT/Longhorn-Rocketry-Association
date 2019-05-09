#include "abc_preconfig.h"
#include <chrono>
#include "flight_sim.h"
#include <fstream>
#include <iostream>
#include "mock_sac_rocket.h"
#include "stats.h"
#include <string>
#include "telemetry.h"
#include <vector>
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

void print_dataset_summary(Dataset &set) {
  std::cout << "SUMMARY\n-------" << std::endl <<
      "mean=" << set.mean() << std::endl <<
      "stdev=" << set.stdev() << std::endl;
}

void print_config(AirbrakeControllerConfiguration &config) {
  std::cout << "CONFIG\n------\n" <<
    "bounds_history_size=" << config.bounds_history_size << "\n" <<
    "enforce_bounds_history_size=" << config.enforce_bounds_history_size << "\n" <<
    "regression_id=" << config.regression_id << "\n" <<
    "bs_profile_velocity_min=" << config.bs_profile_velocity_min << "\n" <<
    "bs_profile_velocity_max=" << config.bs_profile_velocity_max << "\n" <<
    "bs_profile_step_min=" << config.bs_profile_step_min << "\n" <<
    "bs_profile_step_max=" << config.bs_profile_step_max << "\n" <<
    "bs_profile_exp=" << config.bs_profile_exp << "\n" <<
    "bsc_history_size=" << config.bsc_history_size << "\n" <<
    "bsc_thresh_osc=" << config.bsc_thresh_osc << "\n" <<
    "bsc_thresh_stb=" << config.bsc_thresh_stb << "\n" <<
    "bsc_down_profile_velocity_min=" << config.bsc_down_profile_velocity_min << "\n" <<
    "bsc_down_profile_velocity_max=" << config.bsc_down_profile_velocity_max << "\n" <<
    "bsc_down_profile_weight_min=" << config.bsc_down_profile_weight_min << "\n" <<
    "bsc_down_profile_weight_max=" << config.bsc_down_profile_weight_max << "\n" <<
    "bsc_down_profile_exp=" << config.bsc_down_profile_exp << "\n" <<
    "bsc_up_profile_velocity_min=" << config.bsc_up_profile_velocity_min << "\n" <<
    "bsc_up_profile_velocity_max=" << config.bsc_up_profile_velocity_max << "\n" <<
    "bsc_up_profile_weight_min=" << config.bsc_up_profile_weight_min << "\n" <<
    "bsc_up_profile_weight_max=" << config.bsc_up_profile_weight_max << "\n" <<
    "bsc_up_profile_exp=" << config.bsc_up_profile_exp << "\n";
}

int main() {
  // Build configurations
  std::vector<AirbrakeControllerConfiguration> configs;

  AirbrakeControllerConfiguration bs_profile_lower;
  bs_profile_lower.bs_profile_velocity_min = 80;
  bs_profile_lower.bs_profile_velocity_max = 320;
  bs_profile_lower.bs_profile_step_min = 0.04;
  bs_profile_lower.bs_profile_step_max = 0.1;

  AirbrakeControllerConfiguration bs_profile_upper;
  bs_profile_upper.bs_profile_velocity_min = 140;
  bs_profile_upper.bs_profile_velocity_max = 400;
  bs_profile_upper.bs_profile_step_min = 0.1;
  bs_profile_upper.bs_profile_step_max = 0.2;

  AirbrakeControllerConfiguration bs_profile_increment;
  bs_profile_increment.bs_profile_velocity_min = 20;
  bs_profile_increment.bs_profile_velocity_max = 20;
  bs_profile_increment.bs_profile_step_min = 0.02;
  bs_profile_increment.bs_profile_step_max = 0.02;

  AirbrakeControllerConfiguration bsc_weight_lower;
  bsc_weight_lower.bsc_down_profile_weight_min = 0.2;
  bsc_weight_lower.bsc_down_profile_weight_max = 0.6;
  bsc_weight_lower.bsc_up_profile_weight_min = 1.2;
  bsc_weight_lower.bsc_up_profile_weight_max = 1.6;

  AirbrakeControllerConfiguration bsc_weight_upper;
  bsc_weight_upper.bsc_down_profile_weight_min = 0.4;
  bsc_weight_upper.bsc_down_profile_weight_max = 0.8;
  bsc_weight_upper.bsc_up_profile_weight_min = 1.4;
  bsc_weight_upper.bsc_up_profile_weight_max = 1.8;

  AirbrakeControllerConfiguration bsc_weight_increment;
  bsc_weight_increment.bsc_down_profile_weight_min = 0.1;
  bsc_weight_increment.bsc_down_profile_weight_max = 0.1;
  bsc_weight_increment.bsc_up_profile_weight_min = 0.1;
  bsc_weight_increment.bsc_up_profile_weight_max = 0.1;

  AirbrakeControllerConfiguration bsc_params_lower;
  bsc_params_lower.bsc_history_size = 10;
  bsc_params_lower.bsc_thresh_osc = 0.4;
  bsc_params_lower.bsc_thresh_stb = 0.1;

  AirbrakeControllerConfiguration bsc_params_upper;
  bsc_params_upper.bsc_history_size = 20;
  bsc_params_upper.bsc_thresh_osc = 0.7;
  bsc_params_upper.bsc_thresh_stb = 0.3;

  AirbrakeControllerConfiguration bsc_params_increment;
  bsc_params_increment.bsc_history_size = 2;
  bsc_params_increment.bsc_thresh_osc = 0.1;
  bsc_params_increment.bsc_thresh_stb = 0.1;

  for (float bspvmin = bs_profile_lower.bs_profile_velocity_min;
      bspvmin <= bs_profile_upper.bs_profile_velocity_min;
      bspvmin += bs_profile_increment.bs_profile_velocity_min)
  for (float bspvmax = bs_profile_lower.bs_profile_velocity_max;
      bspvmax <= bs_profile_upper.bs_profile_velocity_max;
      bspvmax += bs_profile_increment.bs_profile_velocity_max)
  for (float bspsmin = bs_profile_lower.bs_profile_step_min;
      bspsmin <= bs_profile_upper.bs_profile_step_min;
      bspsmin += bs_profile_increment.bs_profile_step_min)
  for (float bspsmax = bs_profile_lower.bs_profile_step_max;
      bspsmax <= bs_profile_upper.bs_profile_step_max;
      bspsmax += bs_profile_increment.bs_profile_step_max)
  for (float bscdwmin = bsc_weight_lower.bsc_down_profile_weight_min;
      bscdwmin <= bsc_weight_upper.bsc_down_profile_weight_min;
      bscdwmin += bsc_weight_increment.bsc_down_profile_weight_min)
  for (float bscdwmax = bsc_weight_lower.bsc_down_profile_weight_max;
      bscdwmax <= bsc_weight_upper.bsc_down_profile_weight_max;
      bscdwmax += bsc_weight_increment.bsc_down_profile_weight_max)
  for (float bscuwmin = bsc_weight_lower.bsc_up_profile_weight_min;
      bscuwmin <= bsc_weight_upper.bsc_up_profile_weight_min;
      bscuwmin += bsc_weight_increment.bsc_up_profile_weight_min)
  for (float bscuwmax = bsc_weight_lower.bsc_up_profile_weight_max;
      bscuwmax <= bsc_weight_upper.bsc_up_profile_weight_max;
      bscuwmax += bsc_weight_increment.bsc_up_profile_weight_max)
  for (float bschs = bsc_params_lower.bsc_history_size;
      bschs <= bsc_params_upper.bsc_history_size;
      bschs += bsc_params_increment.bsc_history_size)
  for (float bscto = bsc_params_lower.bsc_thresh_osc;
      bscto <= bsc_params_upper.bsc_thresh_osc;
      bscto += bsc_params_increment.bsc_thresh_osc)
  for (float bscts = bsc_params_lower.bsc_thresh_stb;
      bscts <= bsc_params_upper.bsc_thresh_stb;
      bscts += bsc_params_increment.bsc_thresh_stb)
  {
    AirbrakeControllerConfiguration config;
    config.bs_profile_velocity_min = bspvmin;
    config.bs_profile_velocity_max = bspvmax;
    config.bs_profile_step_min = bspsmin;
    config.bs_profile_step_max = bspsmax;

    config.bsc_history_size = bschs;
    config.bsc_thresh_osc = bscto;
    config.bsc_thresh_stb = bscts;

    config.bsc_down_profile_weight_min = bscdwmin;
    config.bsc_down_profile_weight_max = bscdwmax;
    config.bsc_up_profile_weight_min = bscuwmin;
    config.bsc_up_profile_weight_max = bscuwmax;

    configs.push_back(config);
  }

  // Target altitudes to test
  float target_altitudes[] = {4650, 4700, 4750};
  int num_target_altitudes = 3;

  // Vehicle and simulator setup
  struct RocketData rocket_data;
  rocket_data.initial_altitude = 0;
  rocket_data.drag_coeff = 0.1;
  rocket_data.radius = 0.1;
  rocket_data.airbrake_area = 0.02;
  rocket_data.burnout_mass = 15;
  rocket_data.burnout_velocity = 343 * 0.86;
  rocket_data.burnout_altitude = 1542;
  FlightSimulator flightsim = FlightSimulator(rocket_data);
  flightsim.set_stop_condition(flightsim::STOP_CONDITION_APOGEE);

  const int FLIGHTS_PER_CONFIG = 10;
  AirbrakeControllerConfiguration best_config;
  Dataset best_config_set;
  int stop_after = 100;

  for (int i = 0; i < configs.size(); i++) {
    Dataset config_altitude_error_set;
    AirbrakeControllerConfiguration config = configs[i];

    config.bounds_history_size = 50;
    config.enforce_bounds_history_size = false;
    config.regression_id = abc::REG_NONE;

    config.bs_profile_exp = -1;

    config.bsc_down_profile_velocity_min = config.bs_profile_velocity_min;
    config.bsc_down_profile_velocity_max = config.bs_profile_velocity_max;
    config.bsc_down_profile_exp = -1;

    config.bsc_up_profile_velocity_min = config.bs_profile_velocity_min;
    config.bsc_up_profile_velocity_max = config.bs_profile_velocity_max;
    config.bsc_up_profile_exp = -1;

    for (int j = 0; j < FLIGHTS_PER_CONFIG; j++) {
      for (int k = 0; k < num_target_altitudes; k++) {
        config.target_altitude = target_altitudes[k];
        MockSacRocket rocket = MockSacRocket(&flightsim, rocket_data,
            new AirbrakeController(config));
        rocket.initialize();
        rocket.set_simulator(&flightsim);
        flightsim.begin(0);
        float t_upd = 0;

        while (flightsim.is_running()) {
          flightsim.advance(0.01);
          float t = flightsim.get_time();
          float t_upd_el = t - t_upd;
          if (t_upd_el >= 0.1) {
            rocket.set_timestep(t);
            rocket.loop();
            t_upd = t;
          }
        }

        rocket.stop();
        config_altitude_error_set.add(config.target_altitude -
            flightsim.get_rocket_altitude());
      }
    }

    std::cout << "Completed simulation " << i << " of " << configs.size()
        << std::endl;

    if (i == 0 || compare_configs(config_altitude_error_set,
        best_config_set)) {
      best_config = config;
      best_config_set = config_altitude_error_set;
    }

    if (i >= stop_after)
      break;
  }

  print_dataset_summary(best_config_set);
  print_config(best_config);
}
