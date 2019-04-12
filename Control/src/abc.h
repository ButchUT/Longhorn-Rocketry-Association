#ifndef CONTROL_ABC_H
#define CONTROL_ABC_H

#include <fstream>
#include "polyreg.h"
#include "abc_profiling.h"
#include <string>
#include "telemetry.h"
#include <vector>

using namespace std;

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
pair<float, float> pbintersect(vector<float> &p1, vector<float> &p2);

class AirbrakeController {
private:
  const float ALTITUDE_TARGET;
  const size_t BOUNDS_HISTORY_SIZE;
  const int POLYNOMIAL_ORDER = 2;

  float timeLast;
  float altMinVelocity, altMaxVelocity;
  float brakeExtension;
  int iterations;
  vector<float> historyTimestamps,altMinHistory, altMaxHistory, aminCoeffs,
    amaxCoeffs;

  BrakeProfile *brakeProfile;
  PolynomialRegression polyreg;
  TelemetryPipeline *telemetry;

public:
  const float BRAKE_LOWER_BOUND = 0.0; // Brakes fully retracted
  const float BRAKE_UPPER_BOUND = 1.0; // Brakes fully extended

  /**
    Creates a new controller with no history.

    @param alitudeTarget target apogee
    @param brakeProfile profile for computing brake step size
    @param boundsHistorySize maximum size of bounds history arrays
  */
  AirbrakeController(float altitudeTarget, BrakeProfile *brakeProfile,
    size_t boundsHistorySize);

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
      - Step the brakes in the direction that will bring the convergence
        point closer to the target altitude
      - Step magnitude is found by plugging rocket velocity into the
        controller's brake profile

    @param t system time
    @param v rocket velocity
    @param altMin minimum altitude (in the event of full brake)
    @param altMin maximum altitude (no brake)
    @return new airbrake position on [BRAKE_LOWER_BOUND, BRAKE_UPPER_BOUND]
  */
  float update(float t, float v, float altMin, float altMax);

  /**
    Sets the stream that update telemetry will be printed to. If stream is
    nullptr, no telemetry will be generated.

    @param stream target stream
  */
  void set_telemetry_pipeline(TelemetryPipeline *pipeline);
};

#endif
