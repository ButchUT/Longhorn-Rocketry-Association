#include "abc.h"
#include "control_util.h"

#include <iostream>
#include <math.h>
#include <string>

bool abc::t_conv_is_realistic(float t_conv, float t_now) {
  return t_conv > 0 &&
         t_conv < 100000;
}

AirbrakeController::AirbrakeController(AirbrakeControllerConfiguration config) :
    CONFIG(config) {
  time_last = alt_min_velocity = alt_max_velocity = NIL;
  brake_extension = 0.0;
  iterations = 0;
  history_timestamps = vector<float>();
  alt_min_history = vector<float>();
  alt_max_history = vector<float>();
  this->brake_step_profile = new BinomialProfile(
      config.bs_profile_velocity_min,
      config.bs_profile_velocity_max,
      config.bs_profile_step_min,
      config.bs_profile_step_max,
      config.bs_profile_exp);
  telemetry = nullptr;
  bsc = new BrakeStepController(config);
  regressor = nullptr;

  int coeff_count = 0;
  if (CONFIG.regression_id == abc::REG_QUAD) {
    regressor = new PolynomialRegressor(CONFIG.bounds_history_size, 2);
    coeff_count = 3;
  } else if (CONFIG.regression_id == abc::REG_EXP) {
    regressor = new ExponentialRegressor(CONFIG.bounds_history_size);
    coeff_count = 2;
  }

  amin_coeffs = vector<float>(coeff_count);
  amax_coeffs = vector<float>(coeff_count);
}

AirbrakeController::~AirbrakeController() {
  delete brake_step_profile;
  if (regressor != nullptr)
    delete regressor;
  delete bsc;
}

float AirbrakeController::get_brake_extension() {
  return brake_extension;
}

float AirbrakeController::update(float t, float v, float altMin, float altMax) {
  // Record bounds history and enforce size limit
  history_timestamps.push_back(t);
  alt_min_history.push_back(altMin);
  alt_max_history.push_back(altMax);

  // Unfortunately, O(N) performance is unavoidable here without rewriting the
  // regression algorithm to use ring buffers, and that's stupid
  if (CONFIG.enforce_bounds_history_size &&
    history_timestamps.size() > CONFIG.bounds_history_size) {

    history_timestamps.erase(history_timestamps.begin());
    alt_min_history.erase(alt_min_history.begin());
    alt_max_history.erase(alt_max_history.begin());
  }

  if (iterations > 0) {
    float dt = t - time_last;
    float convergence_altitude = NIL;
    float time_of_convergence = NIL;

    // Approximate the velocity of each bound
    float potential_alt_min_velocity = (altMin -
        alt_min_history[iterations - 1]) / dt;
    float potential_alt_max_velocity = (altMax -
        alt_max_history[iterations - 1]) / dt;

    // Only use bound velocities that are in the right direction
    if (potential_alt_min_velocity >= 0)
        alt_min_velocity = potential_alt_min_velocity;
    if (potential_alt_max_velocity <= 0)
        alt_max_velocity = potential_alt_max_velocity;

    // If valid velocities were computed at some point, proceed with a linear
    // convergence calculation
    if (alt_min_velocity != NIL && alt_max_velocity != NIL) {
      float time_to_convergence = (altMax - altMin) /
          (alt_min_velocity - alt_max_velocity);
      convergence_altitude = altMax + alt_max_velocity * time_to_convergence;
      time_of_convergence = t + time_to_convergence;
    }

    // If a valid convergence time was calculated at some point and the history
    // size is satisfactory, polynomial regression will give a second opinion
    // on convergence time
    if (CONFIG.regression_id != abc::REG_NONE && time_of_convergence != NIL &&
      iterations >= CONFIG.bounds_history_size) {
      float sol_final = NIL, sol_convergence_altitude = NIL;
      vector<float> sols;
      // Fit functions, compute intersection(s)
      regressor->fit(history_timestamps, alt_min_history, amin_coeffs);
      regressor->fit(history_timestamps, alt_max_history, amax_coeffs);
      regressor->intersect(amin_coeffs, amax_coeffs, sols);

      if (CONFIG.regression_id == abc::REG_QUAD) {
        // For quadratic regression, proposed solution is whichever is closest
        // to the slope approximation's guess
        float t_conv1_err = fabs(time_of_convergence - sols[0]);
        float t_conv2_err = fabs(time_of_convergence - sols[1]);
        sol_final = t_conv1_err < t_conv2_err ? sols[0] : sols[1];
        float t1 = sol_final, t2 = t1 * t1;
        sol_convergence_altitude = amin_coeffs[2] * t2 + amin_coeffs[1] * t1 +
            amin_coeffs[0];
      } else if (CONFIG.regression_id == abc::REG_EXP) {
        // For exponential regression, there is only one solution
        sol_final = sols[0];
        sol_convergence_altitude = amin_coeffs[1] * pow(EULERS_NUMBER,
            amin_coeffs[0] * sol_final);
      }

      // If the time predicted with regression is realistic, use it
      if (abc::t_conv_is_realistic(sol_final, t)) {
        time_of_convergence = sol_final;
        convergence_altitude = sol_convergence_altitude;
      }
    }

    // If convergence was successfully calculated, nudge the brakes in
    // the right direction
    if (convergence_altitude != NIL) {
      float error = convergence_altitude - CONFIG.target_altitude;
      float brake_step = brake_step_profile->get(v) * (error < 0 ? -1 : 1);
      brake_step = bsc->update(brake_step, error, v);
      brake_extension = util::fconstrain(brake_extension + brake_step,
          BRAKE_LOWER_BOUND, BRAKE_UPPER_BOUND);

      if (telemetry != nullptr)
        telemetry->sendln("aconv", std::to_string(convergence_altitude));
    } else
      if (telemetry != nullptr)
        telemetry->sendln("aconv", "0");
  }

  if (telemetry != nullptr)
    telemetry->sendln("bsc_weight", std::to_string(bsc->get_weight()));

  time_last = t;
  iterations++;

  return brake_extension;
}

void AirbrakeController::set_telemetry_pipeline(TelemetryPipeline *pipeline) {
  telemetry = pipeline;
}
