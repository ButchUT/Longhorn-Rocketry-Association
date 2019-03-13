# Simulator

A simple 1D flight simulator for sensor data mocking.

`RocketData` is configured and passed to a `FlightSimulator`, which is then `advance`d incrementally over small timesteps. Optionally, `NoiseGenerator`s can be used to distort simulator outputs such as the rocket's acceleration vector.

See `test.cc` for a usage example.

Mock sensors that require a simulator for generating logical data should provide a means of attaching a `FlightSimulator*` to themselves, which the sensor will then poll for relevant data on `Read` calls. Noise should be configured in the simulator rather than the sensor.
