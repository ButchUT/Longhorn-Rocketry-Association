#include "abc.h"
#include <math.h>

#define NIL 0x7FC00000

float fconstrain(float f, float lower, float upper)
{
    return f < lower ? lower : (f > upper ? upper : f);
}

float pbintersect(vector<float> &p1, vector<float> &p2)
{
    float a = p1[2] - p2[2];
    float b = p1[1] - p2[1];
    float c = p1[0] - p2[0];
    float discrim = b * b - 4 * a * c;

    // Parabolas do not intersect
    if (discrim < 0)
        return NIL;

    float droot = sqrt(discrim);
    float sol1 = (-b + droot) / (2 * a), sol2 = (-b - droot) / (2 * a);

    // The larger of the two solutions is the most likely place of convergence
    return sol1 < sol2 ? sol2 : sol1;
}

AirBrakeController::AirBrakeController(float altTarget, BrakeProfile
    *brakeProfile) : ALTITUDE_TARGET(altTarget),
    polyreg(PolynomialRegression(BOUNDS_HISTORY_SIZE, POLYNOMIAL_ORDER))
{
    timeLast = altMinVelocity = altMaxVelocity = NIL;
    brakeExtension = 0.0;
    iterations = 0;
    historyTimestamps = vector<float>();
    altMinHistory = vector<float>();
    altMaxHistory = vector<float>();
    aminCoeffs = vector<float>(POLYNOMIAL_ORDER + 1);
    amaxCoeffs = vector<float>(POLYNOMIAL_ORDER + 1);
    this->brakeProfile = brakeProfile;
    aconv_out = nullptr;
}

AirBrakeController::~AirBrakeController()
{
    delete brakeProfile;
}

float AirBrakeController::get_brake_extension()
{
    return brakeExtension;
}

float AirBrakeController::update(float t, float v, float altMin, float altMax)
{
    // Record bounds history and enforce size limit
    historyTimestamps.push_back(t);
    altMinHistory.push_back(altMin);
    altMaxHistory.push_back(altMax);

    // Unfortunately, O(N) performance is unavoidable here without rewriting the
    // regression algorithm to use ring buffers, and that's stupid
    if (historyTimestamps.size() > BOUNDS_HISTORY_SIZE)
    {
        historyTimestamps.erase(historyTimestamps.begin());
        altMinHistory.erase(altMinHistory.begin());
        altMaxHistory.erase(altMaxHistory.begin());
    }

    if (iterations > 0)
    {
        float dt = t - timeLast;
        float convergenceAltitude;

        // If the history has yet to fill up, base convergence computation
        // on slope approximations
        if (iterations < BOUNDS_HISTORY_SIZE)
        {
            // Compute velocities of altitude bounds
            float dt = t - timeLast;
            float potentialAltMinVelocity = (altMin -
                altMinHistory[iterations - 1]) / dt;
            float potentialAltMaxVelocity = (altMax -
                altMaxHistory[iterations - 1]) / dt;

            // Only use bound velocities that are in the right direction
            if (potentialAltMinVelocity >= 0)
                altMinVelocity = potentialAltMinVelocity;
            if (potentialAltMaxVelocity <= 0)
                altMaxVelocity = potentialAltMaxVelocity;

            // If valid velocities were computed at some point, proceed with
            // convergence calculation
            if (altMinVelocity != NIL && altMaxVelocity != NIL)
            {
                float timeToConvergence = (altMax - altMin) /
                    (altMinVelocity - altMaxVelocity);
                convergenceAltitude = altMax + altMaxVelocity *
                    timeToConvergence;
            }

        // History has topped off; use polynomial regression to predict
        // convergence
        } else {
            polyreg.polyreg(historyTimestamps, altMinHistory, aminCoeffs);
            polyreg.polyreg(historyTimestamps, altMaxHistory, amaxCoeffs);
            float timeOfConvergence = pbintersect(aminCoeffs, amaxCoeffs);

            if (timeOfConvergence > t)
            {
                float t = timeOfConvergence, t2 = t * t;
                convergenceAltitude = aminCoeffs[2] * t2 + aminCoeffs[1] * t +
                    aminCoeffs[0];
            }
        }

        // If convergence was successfully calculated, nudge the brakes in
        // the right direction
        if (convergenceAltitude != NIL)
        {
            float error = convergenceAltitude - ALTITUDE_TARGET;
            float brakeStep = brakeProfile->get_step_size(v) *
                (error < 0 ? -1 : 1);
            brakeExtension = fconstrain(brakeExtension + brakeStep,
                BRAKE_LOWER_BOUND, BRAKE_UPPER_BOUND);
            (*aconv_out) << convergenceAltitude << std::endl;
        } else
          (*aconv_out) << 0 << std::endl;
    }

    timeLast = t;
    iterations++;

    return brakeExtension;
}

void AirBrakeController::attach_aconv_out(std::ofstream &stream)
{
    aconv_out = &stream;
}
