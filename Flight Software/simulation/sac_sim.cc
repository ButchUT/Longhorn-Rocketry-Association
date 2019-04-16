#include "abc_preconfig.h"
#include <chrono>
#include "flight_sim.h"
#include <fstream>
#include <iostream>
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

void expreg_test() {
  std::vector<float> x, y;
  std::vector<float> coeffs(2);

  x.push_back(0);
  x.push_back(1);
  x.push_back(2);
  x.push_back(3);

  y.push_back(100);
  y.push_back(75);
  y.push_back(68);
  y.push_back(63);

  ExponentialRegressor reg(3);
  reg.fit(x, y, coeffs);

  std::cout << coeffs[0] << " " << coeffs[1] << std::endl;
}

void verlet_test() {
  struct InitializationData vinit;
  vinit.initial_value = 0;
  vinit.start_time = 0;
  vinit.initial_velocity = 200;
  VerletIntegrator vint = VerletIntegrator(vinit);

  struct AccelerationCalculationData vacc;
  vacc.drag_coefficient = 0.1;
  vacc.radius = 0.1;
  vacc.base_mass = 45;

  double alt1 = vint.SimulateApogeeEuler(0.01, vacc);
  double alt2 = vint.SimulateApogeeVerlet(0.01, vacc);

  std::cout << alt1 << std::endl << alt2 << std::endl;
}

int main() {
  expreg_test();

  /*// Configure rocket
  struct RocketData rdata;
  rdata.initial_altitude = 0;
  rdata.drag_coeff = 0.1;
  rdata.radius = 0.1;
  rdata.airbrake_area = 0.5;
  rdata.burnout_mass = 45;
  rdata.burnout_velocity = 1125.33 * 0.86;
  rdata.burnout_altitude = 1500;

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

  telemetry.close();*/
}
