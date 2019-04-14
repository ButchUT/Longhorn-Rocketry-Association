#include "acceleration_calculator.h"
#include <assert.h>
#include "density_calculator.h"
#include "flightsim.h"
#include <math.h>

#define GRAVITY 9.80665

void flightsim::assert_valid_axis(int axis) {
  assert(axis == flightsim::X_AXIS || axis == flightsim::Y_AXIS ||
    axis == flightsim::Z_AXIS);
}

void flightsim::assert_valid_stop_condition(int cond) {
  assert(cond == flightsim::STOP_CONDITION_APOGEE ||
    cond == flightsim::STOP_CONDITION_CRASH ||
    cond == flightsim::STOP_CONDITION_TIME);
}

FlightSimulator::FlightSimulator(const struct RocketData &rdata) :
  rocket_data(rdata) {
  sim_running = false;
  stop_condition = flightsim::STOP_CONDITION_APOGEE;
  gen = new UniformNoiseGenerator(-3, 3);
}

FlightSimulator::~FlightSimulator() {}

void FlightSimulator::begin(float time_initial, float duration) {
  t_0 = time_initial;
  t_c = time_initial;
  t_f = time_initial + duration;
  airbrake_extension = 0;
  sim_running = true;
  rocket_velocity = rocket_data.burnout_velocity;
  rocket_altitude = rocket_data.burnout_altitude;
  rocket_acceleration = 0;
}

bool FlightSimulator::is_running() {
  return sim_running;
}

void FlightSimulator::advance(float delta_t) {
  if (!is_running())
    return;

  t_c += delta_t;
  if (sim_running && stop_condition == flightsim::STOP_CONDITION_TIME &&
    t_c >= t_f)
    sim_running = false;

  // Compute drag force
  float airbrake_area = airbrake_extension * rocket_data.airbrake_area;
  float r = sqrt(rocket_data.radius * rocket_data.radius + airbrake_area /
    M_PI);
  rocket_acceleration = (float)calculate_acceleration(
    NULL,
    0,
    rocket_data.burnout_mass,
    rocket_velocity,
    rocket_altitude,
    0,
    r,
    rocket_data.drag_coeff,
    0
  ) + gen->gen(t_c);
  rocket_velocity += rocket_acceleration * delta_t;
  rocket_altitude += rocket_velocity * delta_t;

  if (rocket_velocity <= 0 &&
    stop_condition == flightsim::STOP_CONDITION_APOGEE)
    sim_running = false;
  else if (rocket_altitude <= 0 &&
    stop_condition == flightsim::STOP_CONDITION_CRASH)
    sim_running = false;
}

float FlightSimulator::get_time() {
  return t_c;
}

void FlightSimulator::set_airbrake_extension(float x) {
  airbrake_extension = x;
}

void FlightSimulator::set_stop_condition(int cond, float t) {
  flightsim::assert_valid_stop_condition(cond);
  stop_condition = cond;
  if (cond == flightsim::STOP_CONDITION_TIME)
    t_f = t;
}

float FlightSimulator::get_air_pressure() {
  return (float)Interp(kPressureTable, kNumPressureTableValues,
    rocket_altitude);
}

float FlightSimulator::get_rocket_velocity() {
  return rocket_velocity;
}

float FlightSimulator::get_rocket_altitude() {
  return rocket_altitude;
}

std::vector<float> FlightSimulator::get_rocket_acceleration() const {
  std::vector<float> accel = std::vector<float>(flightsim::AXES_COUNT);
  // Z axis receives the full magnitude of the acceleration vector because
  // the simulator is only 1D. Noise is still applied to the other axes if
  // configured
  accel[flightsim::Z_AXIS] = rocket_acceleration;
  return accel;
}

std::vector<float> FlightSimulator::get_rocket_gyro() const {
  // True vector is all 0s; at the moment, the simulated rocket is "perfect"
  // and cannot roll. Noise is added regardless
  std::vector<float> gyro = std::vector<float>(flightsim::AXES_COUNT);
  return gyro;
}
