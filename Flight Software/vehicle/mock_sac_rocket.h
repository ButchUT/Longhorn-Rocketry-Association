#ifndef VEHICLE_MOCK_SAC_ROCKET
#define VEHICLE_MOCK_SAC_ROCKET

#include "abc.h"
#include "flightcomp.h"
#include "flightsim.h"
#include "mock_barometer_mpl.h"
#include "mock_imu_mpl.h"
#include "telemetry.h"
#include "verlet_integrator.h"

class MockSacRocket : public FlightComputerFrame {
public:
  MockSacRocket(FlightSimulator *sim, RocketData rdata);

  ~MockSacRocket();

  void setTimestep(float t);

  void setSimulator(FlightSimulator *sim);

  void setTelemetryPipeline(TelemetryPipeline *pipeline);

  void initialize();

  void loop();

  void stop();

protected:
  AirbrakeController *abc;
  FlightSimulator *flightsim;
  MockBarometerWrapper *barometer;
  MockImuWrapper *imu;
  RocketData rocket_data;
  TelemetryPipeline *telemetry;
  float timestep;
};

#endif
