#ifndef VEHICLE_ROCKET_H
#define VEHICLE_ROCKET_H

/**
  @brief Abstraction of a rocket. Encapsulates sensors and systems into a
  common class
*/
class Rocket {
public:
  /**
    @brief Initialize the rocket and all of its subsystems
  */
  Rocket();

  /**
    @brief Tear down the rocket
  */
  ~Rocket();

  /**
    @brief Iterative method for collecting data, updating actuators, piping
    telemetry, etc.
  */
  virtual void loop() = 0;
};

#endif
