#include "abc_bsc.h"
#include "control_util.h"
#include <iostream>

BrakeStepController::BrakeStepController(
    AirbrakeControllerConfiguration config) {
  this->history_size = config.bsc_history_size;
  thresh_osc = config.bsc_thresh_osc;
  thresh_stb = config.bsc_thresh_stb;
  up_profile = new BinomialProfile(
      config.bsc_up_profile_velocity_min,
      config.bsc_up_profile_velocity_max,
      config.bsc_up_profile_weight_min,
      config.bsc_up_profile_weight_max,
      config.bsc_up_profile_exp);
  down_profile = new BinomialProfile(
      config.bsc_down_profile_velocity_min,
      config.bsc_down_profile_velocity_max,
      config.bsc_down_profile_weight_min,
      config.bsc_down_profile_weight_max,
      config.bsc_down_profile_exp);
  weight = 1.0;
}

BrakeStepController::~BrakeStepController() {
  delete up_profile;
  delete down_profile;
}

float BrakeStepController::update(float step_size, float altitude_error,
    float velocity) {
  // Maintain error history
  error_history.push_back(altitude_error);
  if (error_history.size() > history_size)
    error_history.erase(error_history.begin());

  // Compute number of sign swaps between subsequent history entries
  int sign_swaps = 0;
  for (int i = 1; i < error_history.size(); i++)
    if (util::fsign(error_history[i]) != util::fsign(error_history[i - 1]))
      sign_swaps++;

  // Throttle weight up or down depending on density of sign swaps
  float swap_ratio = sign_swaps * 1.0 / error_history.size();
  if (swap_ratio >= thresh_osc) {
    float down_weight = down_profile->get(velocity);
    weight = util::fconstrain(weight * down_weight, 0, 1);
  } else if (swap_ratio < thresh_stb) {
    float up_weight = up_profile->get(velocity);
    weight = util::fconstrain(weight * up_weight, 0, 1);
  }

  return step_size * weight;
}

float BrakeStepController::get_weight() {
  return weight;
}
