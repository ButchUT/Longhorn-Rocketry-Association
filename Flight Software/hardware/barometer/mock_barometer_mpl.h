#ifndef HARDWARE_MOCK_BAROMETER_MPL_H
#define HARDWARE_MOCK_BAROMETER_MPL_H

#include "barometer.h"
#include "flightsim.h"
#include "noise.h"
#include <string>

class MockBarometerWrapper : public Barometer {
public:
  MockBarometerWrapper() {}

  void initialize();

  /**
    @brief Get most recent sensor data
  */
  struct BarometerData read() const;

  /**
    @brief Sets the time value used for noise generation
  */
  void setTimestep(float t);

  /**
    @brief Attaches a flight simulator from which to derive data for readings
    (mandatory)
  */
  void setSimulator(FlightSimulator *sim);

  /**
    @brief Attaches a noise source to future pressure readings
  */
  void setPressureNoise(NoiseGenerator *gen);

  /**
    @brief Attaches a noise source to future altimeter readings
  */
  void setAltitudeNoise(NoiseGenerator *gen);

  /**
    @brief Attaches a noise source to future thermometer readings
  */
  void setTemperatureNoise(NoiseGenerator *gen);

  /**
    @brief Attaches a noise source to all future readings
  */
  void setNoise(NoiseGenerator *gen);

protected:
  float timestep;
  FlightSimulator *flightsim;
  NoiseGenerator *pressure_noise, *altitude_noise, *temperature_noise;
};

#endif
