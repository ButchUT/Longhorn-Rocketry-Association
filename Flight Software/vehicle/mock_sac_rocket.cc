#include "abc_preconfig.h"
#include <math.h>
#include "mock_sac_rocket.h"
#include "noise.h"
#include <string>

MockSacRocket::MockSacRocket(FlightSimulator *sim, RocketData rdata) {
  this->rocket_data = rdata;

  barometer = new MockBarometerWrapper();
  barometer->setSimulator(sim);
  UniformNoiseGenerator *barometer_noise = new UniformNoiseGenerator(-10, 10);
  barometer->setAltimeterNoise(barometer_noise);

  imu = new MockImuWrapper();
  imu->setSimulator(sim);
  UniformNoiseGenerator *imu_noise = new UniformNoiseGenerator(-5, 5);
  imu->setAccelerometerNoise(imu_noise);

  abc = make_2019_sac_abc();
}

MockSacRocket::~MockSacRocket() {
  delete barometer;
  delete imu;
}

void MockSacRocket::setTimestep(float t) {
  timestep = t;
}

void MockSacRocket::setSimulator(FlightSimulator *sim) {
  flightsim = sim;
}

void MockSacRocket::setTelemetryPipeline(TelemetryPipeline *pipeline) {
  telemetry = pipeline;
  abc->setTelemetryPipeline(pipeline);
}

void MockSacRocket::loop() {
  struct ImuData imu_data = imu->read();
  struct BarometerData baro_data = barometer->read();

  float altitude = baro_data.altitude;
  float velocity = flightsim->get_rocket_velocity(); // TODO: stupid
  float acceleration = imu_data.az;

  if (velocity < 0)
    apogee = true;

  struct AccelerationCalculationData acalc_data;
  acalc_data.drag_coefficient = rocket_data.drag_coeff;
  acalc_data.radius = rocket_data.radius;
  acalc_data.base_mass = rocket_data.burnout_mass;

  // Integrate lower altitude curve
  struct InitializationData vint_data;
  vint_data.initial_value = altitude;
  vint_data.initial_velocity = velocity;
  vint_data.start_time = timestep;
  float r_big = sqrt(rocket_data.radius * rocket_data.radius +
    rocket_data.airbrake_area / M_PI);
  acalc_data.radius = r_big;
  VerletIntegrator vint = VerletIntegrator(vint_data);
  float alt_min = (float)vint.SimulateApogee(0.01, acalc_data);

  // Integrate upper altitude curve
  acalc_data.radius = rocket_data.radius;
  float alt_max = (float)vint.SimulateApogee(0.01, acalc_data);

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

bool MockSacRocket::hit_apogee() {
  return apogee;
}
