/*
  Brake step control used to reduce oscillation in brake profiles.
*/
#ifndef CONTROL_ABC_BSC_H
#define CONTROL_ABC_BSC_H

#include "abc_config.h"
#include "control_profiling.h"

#include <vector>

/*
  A standalone controller that receives brake steps and modifies them.
*/
class BrakeStepController {
protected:
  BinomialProfile *up_profile, *down_profile;
  std::vector<float> error_history;
  float weight, thresh_osc, thresh_stb;
  std::size_t history_size;

public:
  /*
    @brief Creates a new controller with some configuration (BSC config is
           included in ABC config)
  */
  BrakeStepController(AirbrakeControllerConfiguration config);

  ~BrakeStepController();

  /*
    Modifies a brake step so as to smooth brake movement.

    @param step_size base step size
    @param altitude_error current difference between projected and target
           altitude
    @param velocity current rocket velocity
    @return modified brake step
  */
  float update(float step_size, float altitude_error, float velocity);

  /*
    @brief Gets the current brake step weight; useful telemetry
  */
  float get_weight();
};

#endif
