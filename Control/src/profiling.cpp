#include <math.h>
#include "profiling.h"

BinomialBrakeProfile::BinomialBrakeProfile(float minBrakeStep,
    float maxBrakeStep, float minInput, float maxInput, float e)
{
    float minip = pow(minInput, e), maxip = pow(maxInput, e);
    a = (minBrakeStep - maxBrakeStep) / (maxip - minip);
    b = maxBrakeStep - a * minip;
    this->minBrakeStep = minBrakeStep;
    this->maxBrakeStep = maxBrakeStep;
    this->minInput = minInput;
    this->maxInput = maxInput;
    this->e = e;
}

float BinomialBrakeProfile::get_step_size(float x)
{
    if (x <= minInput)
        return maxBrakeStep;
    else if (x >= maxInput)
        return minBrakeStep;

    return a * pow(x, e) + b;
}
