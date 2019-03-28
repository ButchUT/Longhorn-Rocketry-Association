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

  // Telemetry
  std::ofstream time_out, brake_out, low_bound_out, high_bound_out, alt_out,
    vel_out, acc_out, aconv_out;
  time_out.open("dat/time.dat");
  brake_out.open("dat/brake.dat");
  low_bound_out.open("dat/lowbound.dat");
  high_bound_out.open("dat/highbound.dat");
  alt_out.open("dat/altitude.dat");
  vel_out.open("dat/velocity.dat");
  acc_out.open("dat/acceleration.dat");
  aconv_out.open("dat/aconv.dat");

  // Configuration
  TelemetryPipeline telemetry = TelemetryPipeline();
  FlightSimulator flightsim = FlightSimulator(rdata);
  MockSacRocket rocket = MockSacRocket(&flightsim, rdata);

  rocket.setTelemetryPipeline(&telemetry);
  rocket.setSimulator(&flightsim);
  flightsim.begin(0, 60);

  telemetry.addPipe("time", &time_out);
  telemetry.addPipe("brake", &brake_out);
  telemetry.addPipe("low_bound", &low_bound_out);
  telemetry.addPipe("high_bound", &high_bound_out);
  telemetry.addPipe("alt", &alt_out);
  telemetry.addPipe("vel", &vel_out);
  telemetry.addPipe("acc", &acc_out);
  telemetry.addPipe("aconv", &aconv_out);

  // Go time
  while (flightsim.is_running()) {
    flightsim.advance(0.1);
    rocket.setTimestep(flightsim.get_time());
    rocket.loop();
    if (rocket.hit_apogee())
      break;
  }

  std::cout << "Rocket hit " << flightsim.get_rocket_altitude() << std::endl;

  telemetry.close();
}
