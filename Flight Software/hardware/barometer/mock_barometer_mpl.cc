#include "mock_barometer_mpl.h"
#include <sstream>

void MockBarometerWrapper::initialize() {}

void MockBarometerWrapper::setTimestep(float t) {
  timestep = t;
}

void MockBarometerWrapper::setSimulator(FlightSimulator *sim) {
  flightsim = sim;
}

void MockBarometerWrapper::setPressureNoise(NoiseGenerator *gen) {
  pressure_noise = gen;
}

void MockBarometerWrapper::setAltitudeNoise(NoiseGenerator *gen) {
  altitude_noise = gen;
}

void MockBarometerWrapper::setTemperatureNoise(NoiseGenerator *gen) {
  temperature_noise = gen;
}

void MockBarometerWrapper::setNoise(NoiseGenerator *gen) {
  setPressureNoise(gen);
  setAltitudeNoise(gen);
  setTemperatureNoise(gen);
}

struct BarometerData MockBarometerWrapper::read() const
{
  struct BarometerData data;

  data.pressure = flightsim->get_air_pressure();
  data.altitude = flightsim->get_rocket_altitude();
  // Temperature?

  return data;
}
