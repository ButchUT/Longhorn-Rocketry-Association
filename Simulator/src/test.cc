#include <iostream>
#include "noise.h"
#include "flightsim.h"
#include <vector>

int main() {
  SinusoidalNoiseGenerator *g1 = new SinusoidalNoiseGenerator(-1, 1, 1);
  UniformNoiseGenerator *g2 = new UniformNoiseGenerator(-0.1, 0.1);
  CompoundNoiseGenerator g3 = CompoundNoiseGenerator(-100, 100);
  g3.add_generator(g1);
  g3.add_generator(g2);

  struct RocketData rdata;
  rdata.initial_altitude = 0;
  rdata.drag_coeff = 0.01;
  rdata.radius = 0.1;
  rdata.airbrake_area = 0.05;
  rdata.burnout_mass = 15;
  rdata.burnout_velocity = 343 * 3;
  rdata.burnout_altitude = 200;

  FlightSimulator sim = FlightSimulator(rdata);

  sim.begin(0, 120);
  sim.set_accel_noise(&g3);
  sim.set_airbrake_extension(1);

  while (sim.is_running()) {
    sim.advance(0.1);
    std::vector<float> accel = sim.get_acceleration();
    std::cout << sim.get_time() << " " << accel[2] << std::endl;
  }
}
