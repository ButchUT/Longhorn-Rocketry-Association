## Overview

A simple 1D flight simulator for sensor data mocking.

`RocketData` is configured and passed to a `FlightSimulator`, which is then `advance`d incrementally over small timesteps. Optionally, `NoiseGenerator`s can be used to distort sensor outputs such as the rocket's acceleration vector. Rocket control code is run independently of the simulator.

See `sac_sim.cc` for an example configuration.

Mock sensors that require a simulator for generating logical data should provide a means of attaching a `FlightSimulator*` to themselves, which the sensor will then poll for relevant data on `read` calls. Noise should be configured in the individual sensor instances rather than the simulator attached to them.

## Configuring the Simulator

A fully-configured simulator for the SAC rocket can be found in `sac_sim.cc`. To build and run it, type `make sac_sim` into a terminal followed by `./sac_sim`. The simulation will run all at once, report the apogee of the flight, and send the rest of the telemetry to `dat/`.

For tweaking the simulation environment, there are three main files to modify:
* `Control/src/abc_preconfig.cc` - A method `make_2019_sac_abc` builds the rocket's airbrake controller. This decides the target apogee and brake step size, among other things
* `mock_sac_rocket.cc` - The `MockSacRocket` class holds the actual control code that interprets sensor data and moves the airbrakes. Mock sensors and simulation noise should be configured in the constructor of this class
* `sac_sim.cc` - Combines simulation and rocket updates into a common loop. Rocket properties like drag coefficient and burnout velocity are also configured here

## Data Visualization

The simulator pipes output to a number of files in `dat/`. Also in this folder is a Python3 script `vis.py` that takes `.dat` file arguments and graphs them with matplot.

For example:
```
python3 vis.py brake.dat
```

The script can also overlay multiple datasets onto the same graph:
```
python3 vis.py lowbound.dat highbound.dat
```

## Todo

* [ ] Altimeter acts to correct acceleration data
* [ ] Atmospheric pressure tables for different climates
* [ ] Barometer readings are corrected using the aforementioned tables
* [ ] Factor in temperature readings in some useful way
* [ ] Stroke length calculator (relate servo angle to airbrake extension percentage)
* [ ] Realtime simulation with GUI and telemetry graphs, ability to stop/start/increment clock
* [ ] All-in-one-place configuration (rocket, airbrake, atmosphere, etc.)
