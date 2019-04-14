/**
 * @file verlet_integrator.h
 * @brief Verlet integration functionality
 *
 * https://en.wikipedia.org/wiki/Verlet_integration
 */

#ifndef _CURVE_GEN_VERLET_INTEGRATOR_H_
#define _CURVE_GEN_VERLET_INTEGRATOR_H_

// The higher this number, more intricate the velocity calculation is
#define PAST_NUM_STEPS 5

#include <cstddef>

/**
 * @brief Structure of data required for initialization for a VerletIntegrator
 * @struct InitializationData verlet_integrator.h "verlet_integrator.h"
 */
struct InitializationData {
  /**
   * @brief Initial altitude for the verlet integrator in meters
   */
  double initial_value;

  double initial_velocity;

  /**
   * @brief Constant used in acceleration error computing when there's not
   *  enough data points
   */
  double acceleration_error_constant;

  /**
   * @brief Start time in seconds
   */
  double start_time;
};

/**
 * @brief Structure of data irrelevant to the VerletIntegrator but required
 *  for the acceleration_calculator
 * @struct AccelerationCalculationData verlet_integrator.h "verlet_integrator.h"
 */
struct AccelerationCalculationData {
  /**
   * @brief Dimensionless drag constant associated with Reynold's number
   */
  double drag_coefficient;

  /**
   * @brief Radius of rocket tube in meters
   */
  double radius;

  /**
   * @brief Mass of fuel-empty rocket in kilograms
   */
  double base_mass;
};

/**
 * @brief Verlet Integrator class for taking a double integral of acceleration
 * to solve for altitude
 * @class
 */
class VerletIntegrator {
public:
  /**
   * @brief Past number of steps to use when calculating velocity
   * @static
   */
  static const unsigned int kPastNumSteps;

  /**
   * @brief Construct a new Verlet Integrator object
   *
   * @param initialization_data Struct of initialization variables
   */
  VerletIntegrator(struct InitializationData initialization_data);

  /**
   * @brief Calculate velocity at the given index in the data array
   *
   * @param data_array Array of calculated altitude data
   * @param data_array_size
   * @param index Location requested for velocity calculation. Must be less
   *  than or equal to `data_array_size`
   * @return double
   */
  double CalculateVelocity(double *data_array, std::size_t data_array_size,
                           unsigned int index, double timestep);

  /**
   * @brief Simulate altitude for the *n* timesteps
   *
   * @param data_array array of data where the results will be stored
   * @param data_array_size size of data array
   */
  void Simulate(double *data_array, std::size_t data_array_size,
                double timestep,
                const struct AccelerationCalculationData &acceleration_data);

  double SimulateApogeeVerlet(double timestep, const struct
    AccelerationCalculationData &acceleration_data);

  /**
   * Generates a curve until negative velocity is encountered. For rockets
   * burning out at around Mach 3, timestep should be 0.01 or smaller for best
   * results.
   *
   * This method uses a high resolution Riemann sum rather than true Verlet
   * integration.
   *
   * @param timestep time resolution
   * @return predicted apogee
   */
  double SimulateApogeeEuler(double timestep, const struct
    AccelerationCalculationData &acceleration_data);

  /**
   * @brief Access teh initial_value property
   *
   * @return double
   */
  double initial_value() const;

  /**
   * @brief Access the initial_velocity property
   *
   * @return double
   */
  double initial_velocity() const;

  /**
   * @brief Access the acceleration_error_constant property
   *
   * @return double
   */
  double acceleration_error_constant() const;

  /**
   * @brief Access the start_time property
   *
   * @return double
   */
  double start_time() const;

private:
  double initial_value_;
  double initial_velocity_;
  double acceleration_error_constant_;
  double start_time_;
};

#endif // _CURVE_GEN_VERLET_INTEGRATOR_H_
