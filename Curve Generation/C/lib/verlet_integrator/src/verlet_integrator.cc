/**
 * @file verlet_integrator.cc
 * @brief Implementation of the verlet integrator
 *
 * Integrates acceleration twice to solve for altitude
 */

#include "verlet_integrator.h"

#include <algorithm> // std::min
#include <cassert>   // assert
#include <cstddef>   // std::size_t
#include <vector>
#include <iostream>

#include "acceleration_calculator.h"

const unsigned int VerletIntegrator::kPastNumSteps = PAST_NUM_STEPS;

VerletIntegrator::VerletIntegrator(struct InitializationData initialization_data)
    : initial_value_(initialization_data.initial_value),
      initial_velocity_(initialization_data.initial_velocity),
      acceleration_error_constant_(initialization_data.acceleration_error_constant),
      start_time_(initialization_data.start_time) {}

double VerletIntegrator::CalculateVelocity(double *data_array,
                                           size_t data_array_size,
                                           unsigned int index,
                                           double timestep) {
  if (index > data_array_size)
    throw "index must be <= data_array_size";

  unsigned int num_samples = std::min(VerletIntegrator::kPastNumSteps, index - 1);
  double sum = 0.0;
  for (unsigned int i = 0; i < num_samples; ++i) {
    sum += (data_array[index - 1 - i] - data_array[index - 2 - i]);
  }

  return sum / (timestep * num_samples);
}

void VerletIntegrator::Simulate(double *data_array, std::size_t data_array_size,
                                double timestep,
                                const struct AccelerationCalculationData &acceleration_data) {
  assert(data_array_size > 2);

  data_array[0] = this->initial_value();
  data_array[1] = data_array[0] + this->initial_velocity();

  for (unsigned int i = 2; i < data_array_size; ++i) {
    double velocity = this->CalculateVelocity(data_array, data_array_size, i, timestep);
    double height = data_array[i - 1];
    double acceleration = calculate_acceleration(
      NULL,
      0,
      acceleration_data.base_mass,
      velocity,
      height,
      this->initial_value(),
      acceleration_data.radius,
      acceleration_data.drag_coefficient,
      0.0);  // Time is irrelevant in acceleration calculation when collected_data array is empty
    double altitude = (2 * height - data_array[i - 2] + acceleration *
      timestep * timestep);

    data_array[i] = altitude;
  }
}

double VerletIntegrator::SimulateApogeeVerlet(double timestep, const struct
  AccelerationCalculationData &acceleration_data) {

  std::vector<double> data;
  data.push_back(initial_value());
  data.push_back(data[0] + initial_velocity() * timestep);

  int i = 2;
  double velocity = initial_velocity();
  double altitude = initial_value();

  while (velocity > 0) {
    velocity = CalculateVelocity((double*)(&data[0]), data.size(), i, timestep);
    altitude = data[i - 1];
    double acceleration = calculate_acceleration(
      NULL,
      0,
      acceleration_data.base_mass,
      velocity,
      altitude,
      initial_value(),
      acceleration_data.radius,
      acceleration_data.drag_coefficient,
      0
    );
    double altitude_new = 2 * altitude - data[i - 2] +
      acceleration * timestep * timestep;
    data.push_back(altitude_new);
    i++;
  }

  return altitude;
}

double VerletIntegrator::SimulateApogeeEuler(double timestep,
  const struct AccelerationCalculationData &acceleration_data) {

  double altitude = initial_value();
  double velocity = initial_velocity();

  while (velocity > 0) {
    double accel = calculate_acceleration(
      NULL,
      0,
      acceleration_data.base_mass,
      velocity,
      altitude,
      0,
      acceleration_data.radius,
      acceleration_data.drag_coefficient,
      0
    );
    velocity += accel * timestep;
    altitude += velocity * timestep;
  }

  return altitude;
}

double VerletIntegrator::initial_value() const {
  return this->initial_value_;
}

double VerletIntegrator::initial_velocity() const {
  return this->initial_velocity_;
}

double VerletIntegrator::acceleration_error_constant() const {
  return this->acceleration_error_constant_;
}

double VerletIntegrator::start_time() const {
  return this->start_time_;
}
