#include <iostream>
#include "noise.h"
#include "flightsim.h"

int main() {
  SinusoidalNoiseGenerator *s1 = new SinusoidalNoiseGenerator(-2, 2, 4);
  UniformNoiseGenerator *s2 = new UniformNoiseGenerator(-2, 2);
  CompoundNoiseGenerator s3 = CompoundNoiseGenerator(-100, 100);
  s3.add_generator(s1);
  s3.add_generator(s2);
  float f1 = s1->gen(1.5);
  float f2 = s2->gen(0);
  float f3 = s3.gen(0);
  std::cout << f1 << std::endl << f2 << std::endl << f3 << std::endl;

  /*struct RocketData rdata;
  rdata.initial_altitude = 0;
  rdata.drag_coeff = 0.1;
  rdata.radius = 0.3;
  rdata.airbrake_area = 0.1;
  rdata.burnout_mass = 10;
  rdata.burnout_velocity = 300;
  rdata.burnout_altitude = 80;

  AirbrakeSimulator sim = AirbrakeSimulator(rdata);
  sim.begin(0, 10);

  const float dt = 0.1;

  while (sim.is_running()) {
    sim.advance(0.1);
  }

  std::cout << "pretty epic" << std::endl;*/
}
