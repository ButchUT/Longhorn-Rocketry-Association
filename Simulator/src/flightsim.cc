#include <assert.h>
#include "constant_area_drag_calculator.h"
#include "flightsim.h"
#include <math.h>

#define GRAVITY 9.80665

void FlightSimulator::assert_valid_axis(int axis) {
  assert(axis == flightsim::X_AXIS || axis == flightsim::Y_AXIS ||
    axis == flightsim::Z_AXIS);
}

FlightSimulator::FlightSimulator(const struct RocketData &rdata) :
  rocket_data(rdata) {
  sim_running = false;
  accel_noise = std::vector<NoiseGenerator*>(3);
  gyro_noise = std::vector<NoiseGenerator*>(3);
}

FlightSimulator::~FlightSimulator() {
  clear_accel_noise();
  clear_gyro_noise();
}

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
  if (sim_running && t_c >= t_f)
    sim_running = false;

  // Compute drag force
  float airbrake_area = airbrake_extension * rocket_data.airbrake_area;
  float r = sqrt(rocket_data.radius * rocket_data.radius + airbrake_area / _PI);
  float force_drag = calculate_drag(rocket_data.initial_altitude,
    rocket_altitude, r, rocket_data.drag_coeff, rocket_velocity);

  // Extrapolate rocket state
  rocket_acceleration = -force_drag / rocket_data.burnout_mass - GRAVITY;
  rocket_velocity += rocket_acceleration * delta_t;
  rocket_altitude += rocket_velocity * delta_t;
}

float FlightSimulator::get_time() {
  return t_c;
}

void FlightSimulator::set_airbrake_extension(float x) {
  airbrake_extension = x;
}

void FlightSimulator::set_accel_noise(int axis, NoiseGenerator *gen) {
  assert_valid_axis(axis);

  accel_noise[axis] = gen;
}

void FlightSimulator::set_accel_noise(NoiseGenerator *gen) {
  for (int i = 0; i < flightsim::AXES_COUNT; i++)
    accel_noise[i] = gen;
}

void FlightSimulator::clear_accel_noise() {
  for (int i = 0; i < flightsim::AXES_COUNT; i++)
    free(accel_noise[i]);
}

void FlightSimulator::set_gyro_noise(int axis, NoiseGenerator *gen) {
  assert_valid_axis(axis);

  gyro_noise[axis] = gen;
}

void FlightSimulator::set_gyro_noise(NoiseGenerator *gen) {
  for (int i = 0; i < flightsim::AXES_COUNT; i++)
    gyro_noise[i] = gen;
}

void FlightSimulator::clear_gyro_noise() {
  for (int i = 0; i < flightsim::AXES_COUNT; i++)
    free(gyro_noise[i]);
}

std::vector<float> FlightSimulator::get_acceleration() const {
  std::vector<float> accel = std::vector<float>(flightsim::AXES_COUNT);
  // Z axis receives the full magnitude of the acceleration vector because
  // the simulator is only 1D. Noise is still applied to the other axes if
  // configured
  accel[flightsim::Z_AXIS] = rocket_acceleration;

  for (int i = 0; i < flightsim::AXES_COUNT; i++)
    accel[i] += accel_noise[i]->gen(t_c);

  return accel;
}

std::vector<float> FlightSimulator::get_gyro() const {
  // True vector is all 0s; at the moment, the simulated rocket is "perfect"
  // and cannot roll. Noise is added regardless
  std::vector<float> gyro = std::vector<float>(flightsim::AXES_COUNT);

  for (int i = 0; i < flightsim::AXES_COUNT; i++)
    gyro[i] += gyro_noise[i]->gen(t_c);

  return gyro;
}
