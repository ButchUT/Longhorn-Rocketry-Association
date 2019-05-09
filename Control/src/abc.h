#ifndef CONTROL_ABC_H
#define CONTROL_ABC_H

#include "abc_bsc.h"
#include "abc_config.h"
#include "control_profiling.h"
#include "regression.h"
#include "telemetry.h"

#include <fstream>
#include <string>
#include <vector>

using namespace std;

namespace abc {
  const int REG_NONE = 0;
  const int REG_QUAD = 1;
  const int REG_EXP = 3;

  /**
    A subjective method of determining if a predicted bound convergence time
    is realistic.

    @param t_conv predicted time of bound convergence
    @param t_now current time
    @return if t_conv is realistic
  */
  bool t_conv_is_realistic(float t_conv, float t_now);
}

class AirbrakeController {
private:
  const float BRAKE_LOWER_BOUND = 0.0; // Brakes fully retracted
  const float BRAKE_UPPER_BOUND = 1.0; // Brakes fully extended
  const AirbrakeControllerConfiguration CONFIG;

  float time_last;
  float alt_min_velocity, alt_max_velocity;
  float brake_extension;
  int iterations;
  vector<float> history_timestamps, alt_min_history, alt_max_history,
    amin_coeffs, amax_coeffs;

  Profile *brake_step_profile;
  Regressor *regressor;
  TelemetryPipeline *telemetry;
  BrakeStepController *bsc;

public:
  /**
    Creates a new controller with no history.

    @param config controller configuration
  */
  AirbrakeController(AirbrakeControllerConfiguration config);

  ~AirbrakeController();

  /**
    Retrieves the last calculated brake extension (absolute brake
    position).

    @return brake extension determined by last update call
  */
  float get_brake_extension();

  /**
    Calculates a new brake extension such that the minimum and maximum
    projected altitudes will converge at the target altitude.

    @param t system time
    @param v rocket velocity
    @param alt_min minimum altitude (in the event of full brake)
    @param alt_max maximum altitude (no brake)
    @return new airbrake position on [BRAKE_LOWER_BOUND, BRAKE_UPPER_BOUND]
  */
  float update(float t, float v, float alt_min, float alt_max);

  /**
    Sets the stream that update telemetry will be printed to. If stream is
    nullptr, no telemetry will be generated.

    @param stream target stream
  */
  void set_telemetry_pipeline(TelemetryPipeline *pipeline);
};

#endif
