#ifndef CONTROL_ABC_H
#define CONTROL_ABC_H

#include <fstream>
#include "regression.h"
#include "abc_profiling.h"
#include <string>
#include "telemetry.h"
#include <vector>

using namespace std;

namespace abc {
  const int REG_NONE = 0;
  const int REG_QUAD = 1;
  const int REG_EXP = 3;

  /**
    All-in-one-place configuration for an AirbrakeController.
  */
  struct AirbrakeControllerConfiguration {
    float target_altitude;
    float min_velocity, max_velocity;
    float min_brake_step, max_brake_step;
    float brake_step_profile_exp;
    int bounds_history_size, regression_id;
    bool enforce_bounds_history_size;
  };

  /**
    Clamps a float between some bounds.

    @param f value to constrain
    @param lower lower bound
    @param upper upper bound
  */
  float fconstrain(float f, float lower, float upper);

  /**
    Computes where two parabolas intersect.

    @param p1 <c, b, a> coefficients of parabola 1
    @param p2 <c, b, a> coefficients of parabola 2
    @return largest solution or NIL if parabolas do not intersect
  */
  pair<float, float> pb_intersect(vector<float> &p1, vector<float> &p2);

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
  const struct abc::AirbrakeControllerConfiguration CONFIG;

  float time_last;
  float alt_min_velocity, alt_max_velocity;
  float brake_extension;
  int iterations;
  vector<float> history_timestamps, alt_min_history, alt_max_history,
    amin_coeffs, amax_coeffs;

  BrakeProfile *brake_profile;
  Regressor *regressor;
  TelemetryPipeline *telemetry;

public:
  /**
    Creates a new controller with no history.

    @param config controller configuration
  */
  AirbrakeController(const abc::AirbrakeControllerConfiguration &config);

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

    Synopsis of the algorithm:
      - Maintain a history of the last N min and max altitudes
      - Fit quadratic functions to each history
      - Find the intersection of those functions to predict bound convergence
      - If the point of intersection is nonexistent or nonsensical, a simple
        slope approximation of each bound's velocity is used to compute
        convergence
      - Step the brakes in the direction that will bring the convergence
        point closer to the target altitude
      - Step magnitude is found by plugging rocket velocity into the
        controller's brake profile

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
