#include "mock_barometer_mpl.h"

#include <sstream>

void MockBarometerWrapper::initialize() {}

void MockBarometerWrapper::set_timestep(float t) {
  timestep = t;
}

void MockBarometerWrapper::set_simulator(FlightSimulator *sim) {
  flightsim = sim;
}

void MockBarometerWrapper::set_pressure_noise(NoiseGenerator *gen) {
  pressure_noise = gen;
}

void MockBarometerWrapper::set_altitude_noise(NoiseGenerator *gen) {
  altitude_noise = gen;
}

void MockBarometerWrapper::set_temperature_noise(NoiseGenerator *gen) {
  temperature_noise = gen;
}

void MockBarometerWrapper::set_noise(NoiseGenerator *gen) {
  set_pressure_noise(gen);
  set_altitude_noise(gen);
  set_temperature_noise(gen);
}

struct BarometerData MockBarometerWrapper::read() const
{
  struct BarometerData data;

  data.pressure = flightsim->get_air_pressure();
  data.altitude = flightsim->get_rocket_altitude();
  // Temperature?

  return data;
}
