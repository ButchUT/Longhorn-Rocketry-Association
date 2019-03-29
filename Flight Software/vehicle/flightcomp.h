#ifndef VEHICLE_FLIGHTCOMP_H
#define VEHICLE_FLIGHTCOMP_H

/**
  @brief Abstraction of a flight computer. Holds sensors, controllers, and
  various methods for directing program flow.
*/
class FlightComputerFrame {
public:
  /**
    @brief Initialize the rocket and all of its subsystems
  */
  FlightComputerFrame() {}

  /**
    @brief Tear down the rocket
  */
  ~FlightComputerFrame() {}

  /**
    @brief One-time initialization called before entering the control loop
  */
  virtual void initialize() {}

  /**
    @brief Iterative method for actual control code. Ideally this method
    represents a single observe-interpret-act cycle. Must be overwritten
  */
  virtual void loop() = 0;

  /**
    @brief One-time stop called after the control loop finishes
  */
  virtual void stop() {}
};

#endif
