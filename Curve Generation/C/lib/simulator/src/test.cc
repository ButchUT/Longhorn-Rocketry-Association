#include <iostream>
#include "noise.h"
#include "flightsim.h"
#include <vector>
#include "verlet_integrator.h"
#include "constant_area_drag_calculator.h"

int main() {
  // Configure rocket
  struct RocketData rdata;
  rdata.initial_altitude = 0;
  rdata.drag_coeff = 0.01;
  rdata.radius = 0.1;
  rdata.airbrake_area = 0.05;
  rdata.burnout_mass = 15;
  rdata.burnout_velocity = 343 * 3;
  rdata.burnout_altitude = 200;

  // Configure integrator
  struct InitializationData vintdata;
  vintdata.initial_value = rdata.initial_altitude;
  vintdata.initial_velocity = rdata.burnout_velocity;
  vintdata.acceleration_error_constant = 0;
  vintdata.start_time = 0;

  VerletIntegrator vint = VerletIntegrator(vintdata);
  FlightSimulator sim = FlightSimulator(rdata);

  // Add a noisy sinusoid to acceleration readings
  SinusoidalNoiseGenerator *g1 = new SinusoidalNoiseGenerator(-1, 1, 1);
  UniformNoiseGenerator *g2 = new UniformNoiseGenerator(-0.1, 0.1);
  CompoundNoiseGenerator g3 = CompoundNoiseGenerator(-100, 100);
  g3.add_generator(g1);
  g3.add_generator(g2);
  sim.set_accel_noise(&g3);

  // Run simulation
  sim.begin(0, 120); // t_0=0, t_f=120

  // Should naturally stop at apogee when velocity first becomes negative and
  // fails an assert statement in the drag calculator
  while (sim.is_running()) {
    // Advance sim
    sim.advance(0.1);
    std::vector<float> accel = sim.get_acceleration();
    std::cout << sim.get_time() << " " << accel[2] << std::endl;

    // TODO: integrate lower and upper trajectories, send to airbrake
    // controller, update airbrake position in simulator
  }
}

void make_curve() {

}
