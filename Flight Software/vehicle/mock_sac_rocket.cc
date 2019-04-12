#include "abc_preconfig.h"
#include <iostream>
#include <math.h>
#include "mock_sac_rocket.h"
#include "noise.h"
#include <string>

MockSacRocket::MockSacRocket(FlightSimulator *sim, RocketData rdata) {
  this->flightsim = sim;
  this->rocket_data = rdata;
  altitude_last = timestep_last = -1;
}

MockSacRocket::~MockSacRocket() {
  delete barometer;
  delete imu;
}

void MockSacRocket::set_timestep(float t) {
  timestep = t;
}

void MockSacRocket::set_simulator(FlightSimulator *sim) {
  flightsim = sim;
}

void MockSacRocket::set_telemetry_pipeline(TelemetryPipeline *pipeline) {
  telemetry = pipeline;
  abc->set_telemetry_pipeline(pipeline);
}

void MockSacRocket::initialize() {
  // Initialize barometer
  barometer = new MockBarometerWrapper();
  barometer->initialize();
  barometer->set_simulator(flightsim);
  UniformNoiseGenerator *barometer_noise = new UniformNoiseGenerator(-10, 10);
  barometer->set_altitude_noise(barometer_noise);

  // Initialize IMU
  imu = new MockImuWrapper();
  imu->initialize();
  imu->set_simulator(flightsim);
  UniformNoiseGenerator *imu_noise = new UniformNoiseGenerator(-5, 5);
  imu->set_acceleration_noise(imu_noise);

  // Build airbrake controller
  abc = make_2019_sac_abc();
}

void MockSacRocket::loop() {
  // Read sensors
  struct BarometerData baro_data = barometer->read();
  float altitude = baro_data.altitude;

  struct ImuData imu_data = imu->read();
  float acceleration = imu_data.az;

  // Control requires a previous iteration to calculate a dx/dt with, so the
  // first call to loop() (when timestep_last == -1) is always skipped
  if (timestep_last != -1) {
    // Compute velocity
    // TODO: Is this the best way? Integrating acceleration is also an option...
    // Perhaps some combination of both?
    float velocity = (altitude - altitude_last) /
      (timestep - timestep_last);

    // Configure Verlet integrator
    struct InitializationData vint_data;
    VerletIntegrator vint = VerletIntegrator(vint_data);
    vint_data.initial_value = altitude;
    vint_data.start_time = timestep;
    vint_data.initial_velocity = velocity;
    vint = VerletIntegrator(vint_data);

    // Parameters for acceleration integration
    struct AccelerationCalculationData acalc_data;
    acalc_data.drag_coefficient = rocket_data.drag_coeff;
    acalc_data.radius = rocket_data.radius;
    acalc_data.base_mass = rocket_data.burnout_mass;

    // Integrate lower altitude curve
    // Radius of a circle containing the nose cone area and the area of the
    // extended airbrakes
    float r_big = sqrt(rocket_data.radius * rocket_data.radius +
      rocket_data.airbrake_area / M_PI);
    acalc_data.radius = r_big;
    float alt_min = (float)vint.SimulateApogeeRiemann(0.01, acalc_data);

    // Integrate upper altitude curve
    acalc_data.radius = rocket_data.radius;
    float alt_max = (float)vint.SimulateApogeeRiemann(0.01, acalc_data);

    // Update airbrake
    float extension = abc->update(timestep, velocity, alt_min, alt_max);
    flightsim->set_airbrake_extension(extension);

    // Telemetry
    telemetry->sendln("time", std::to_string(timestep));
    telemetry->sendln("brake", std::to_string(extension));
    telemetry->sendln("low_bound", std::to_string(alt_min));
    telemetry->sendln("high_bound", std::to_string(alt_max));
    telemetry->sendln("alt", std::to_string(altitude));
    telemetry->sendln("vel", std::to_string(velocity));
    telemetry->sendln("acc", std::to_string(acceleration));
  }

  altitude_last = altitude;
  timestep_last = timestep;
}

void MockSacRocket::stop() {}
