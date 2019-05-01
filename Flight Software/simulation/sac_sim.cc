#include <chrono>
#include "control_profiling.h"
#include "flight_sim.h"
#include "abc_preconfig.h"
#include <fstream>
#include <iostream>
#include "control_profiling.h"
#include "mock_sac_rocket.h"
#include "regression.h"
#include <string>
#include "telemetry.h"
#include <vector>
#include "verlet_integrator.h"

double time() {
    chrono::milliseconds ms = chrono::duration_cast<chrono::milliseconds>(
        chrono::system_clock::now().time_since_epoch());
    return ms.count() / 1000.0;
}

int main() {
  // Configure rocket
  struct RocketData rdata;
  rdata.initial_altitude = 0;
  rdata.drag_coeff = 0.1;
  rdata.radius = 0.1;
  rdata.airbrake_area = 0.02;
  rdata.burnout_mass = 15;
  rdata.burnout_velocity = 343 * 0.86;
  rdata.burnout_altitude = 1542;

  // Configuration
  TelemetryPipeline telemetry = TelemetryPipeline();
  FlightSimulator flightsim = FlightSimulator(rdata);
  MockSacRocket rocket = MockSacRocket(&flightsim, rdata, make_2019_sac_abc());

  telemetry.open_pipe("time", "dat/time.dat");
  telemetry.open_pipe("brake", "dat/brake.dat");
  telemetry.open_pipe("low_bound", "dat/lowbound.dat");
  telemetry.open_pipe("high_bound", "dat/highbound.dat");
  telemetry.open_pipe("alt", "dat/altitude.dat");
  telemetry.open_pipe("vel", "dat/velocity.dat");
  telemetry.open_pipe("acc", "dat/acceleration.dat");
  telemetry.open_pipe("aconv", "dat/aconv.dat");
  telemetry.open_pipe("bsc_weight", "dat/bsc_weight.dat");

  rocket.initialize();
  rocket.set_telemetry_pipeline(&telemetry);
  rocket.set_simulator(&flightsim);

  flightsim.set_stop_condition(flightsim::STOP_CONDITION_APOGEE);
  flightsim.begin(0);

  double control_time_total = 0;
  int iterations = 0;

  // Go time
  while (flightsim.is_running()) {
    flightsim.advance(0.1);
    rocket.set_timestep(flightsim.get_time());
    double control_time_start = time();
    rocket.loop();
    control_time_total += time() - control_time_start;
    iterations++;
  }

  rocket.stop();

  std::cout << "FLIGHT SUMMARY" << std::endl << "--------------" << std::endl <<
      "Duration of ascent: " << flightsim.get_time() << std::endl <<
      "Avg. time spent in control: " << control_time_total / iterations << std::endl <<
      "Rocket apogee: " << flightsim.get_rocket_altitude() << std::endl;

  telemetry.close();
}
