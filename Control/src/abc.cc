#include "abc.h"
#include <math.h>
#include <string>
#include <iostream>

#define NIL 0x7FC00000 // Standard NaN value used to indicate "no solution"

float abc::fconstrain(float f, float lower, float upper) {
  return f < lower ? lower : (f > upper ? upper : f);
}

pair<float, float> abc::pb_intersect(vector<float> &p1, vector<float> &p2) {
  float a = p1[2] - p2[2];
  float b = p1[1] - p2[1];
  float c = p1[0] - p2[0];
  float discrim = b * b - 4 * a * c;

  // Parabolas do not intersect
  if (discrim < 0)
    return pair<float, float>(NIL, NIL);

  float droot = sqrt(discrim);
  float sol1 = (-b + droot) / (2 * a), sol2 = (-b - droot) / (2 * a);

  return pair<float, float>(sol1, sol2);
}

bool abc::t_conv_is_realistic(float t_conv, float t_now) {
  return t_conv > 0 &&
         t_conv < 100000;
}

AirbrakeController::AirbrakeController(
  const abc::AirbrakeControllerConfiguration &config) :
  CONFIG(config),
  regressor(PolynomialRegression(config.bounds_history_size,
    POLYNOMIAL_ORDER)) {

  time_last = alt_min_velocity = alt_max_velocity = NIL;
  brake_extension = 0.0;
  iterations = 0;
  history_timestamps = vector<float>();
  alt_min_history = vector<float>();
  alt_max_history = vector<float>();
  amin_coeffs = vector<float>(POLYNOMIAL_ORDER + 1);
  amax_coeffs = vector<float>(POLYNOMIAL_ORDER + 1);
  this->brake_profile = new BinomialBrakeProfile(
    CONFIG.min_brake_step,
    CONFIG.max_brake_step,
    CONFIG.min_velocity,
    CONFIG.max_velocity,
    CONFIG.brake_step_profile_exp
  );
  telemetry = nullptr;
}

AirbrakeController::~AirbrakeController() {
  delete brake_profile;
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
    if (CONFIG.use_polyreg && time_of_convergence != NIL &&
      iterations >= CONFIG.bounds_history_size) {

      // Fit parabolas to each bound history and find their intersections
      regressor.polyreg(history_timestamps, alt_min_history, amin_coeffs);
      regressor.polyreg(history_timestamps, alt_max_history, amax_coeffs);
      pair<float, float> sols = abc::pb_intersect(amin_coeffs, amax_coeffs);

      // Proposed solution is whichever polyreg zero was closest to the linear
      // convergence calculation's prediction
      float t_conv1_err = fabs(time_of_convergence - sols.first);
      float t_conv2_err = fabs(time_of_convergence - sols.second);
      float sol = t_conv1_err < t_conv2_err ? sols.first : sols.second;

      // If the time predicted with regression is realistic, use it
      if (abc::t_conv_is_realistic(sol, t)) {
        float old_alt = convergence_altitude;
        time_of_convergence = sol;
        float t1 = time_of_convergence, t2 = t1 * t1;
        convergence_altitude = amin_coeffs[2] * t2 + amin_coeffs[1] * t1 +
          amin_coeffs[0];
      }
    }

    // If convergence was successfully calculated, nudge the brakes in
    // the right direction
    if (convergence_altitude != NIL) {
      float error = convergence_altitude - CONFIG.target_altitude;
      float brake_step = brake_profile->get_step_size(v) *
        (error < 0 ? -1 : 1);
      brake_extension = abc::fconstrain(brake_extension + brake_step,
        BRAKE_LOWER_BOUND, BRAKE_UPPER_BOUND);

      if (telemetry != nullptr)
        telemetry->sendln("aconv", std::to_string(convergence_altitude));
    } else
      if (telemetry != nullptr)
        telemetry->sendln("aconv", "0");
  }

  time_last = t;
  iterations++;

  return brake_extension;
}

void AirbrakeController::set_telemetry_pipeline(TelemetryPipeline *pipeline) {
  telemetry = pipeline;
}
