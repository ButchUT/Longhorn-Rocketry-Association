#include "flightsim.h"
#include <fstream>
#include <iostream>
#include "mock_sac_rocket.h"
#include <string>
#include "telemetry.h"

int main() {
  // Configure rocket
  struct RocketData rdata;
  rdata.initial_altitude = 0;
  rdata.drag_coeff = 0.01;
  rdata.radius = 0.1;
  rdata.airbrake_area = 0.05;
  rdata.burnout_mass = 15;
  rdata.burnout_velocity = 343;
  rdata.burnout_altitude = 200;

  // Configuration
  TelemetryPipeline telemetry = TelemetryPipeline();
  FlightSimulator flightsim = FlightSimulator(rdata);
  MockSacRocket rocket = MockSacRocket(&flightsim, rdata);

  telemetry.open_pipe("time", "dat/time.dat");
  telemetry.open_pipe("brake", "dat/brake.dat");
  telemetry.open_pipe("low_bound", "dat/lowbound.dat");
  telemetry.open_pipe("high_bound", "dat/highbound.dat");
  telemetry.open_pipe("alt", "dat/altitude.dat");
  telemetry.open_pipe("vel", "dat/velocity.dat");
  telemetry.open_pipe("acc", "dat/acceleration.dat");
  telemetry.open_pipe("aconv", "dat/aconv.dat");

  rocket.initialize();
  rocket.setTelemetryPipeline(&telemetry);
  rocket.setSimulator(&flightsim);

  flightsim.set_stop_condition(flightsim::STOP_CONDITION_APOGEE);
  flightsim.begin(0);

  // Go time
  while (flightsim.is_running()) {
    flightsim.advance(0.1);
    rocket.setTimestep(flightsim.get_time());
    rocket.loop();
  }

  rocket.stop();

  std::cout << "Rocket hit " << flightsim.get_rocket_altitude() << std::endl;

  telemetry.close();
}
