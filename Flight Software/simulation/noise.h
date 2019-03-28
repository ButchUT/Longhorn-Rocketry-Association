#ifndef FLIGHTSIM_NOISE_H
#define FLIGHTSIM_NOISE_H

#include <random>
#include <vector>

/**
  Abstraction of a noise source with a lower and upper bound.
*/
class NoiseGenerator {
protected:
  float low, high;

public:
  /**
    Establishes the generator bounds.

    @param l lower bound
    @param h upper bound
  */
  NoiseGenerator(float l, float h);

  /**
    Prompts the generator for a single noise value.

    @param x independent variable that an implementer may relate to output
    @return noise
  */
  virtual float gen(float x) = 0;
};

/**
  Generates uniform noise on an interval.
*/
class UniformNoiseGenerator : public NoiseGenerator {
protected:
  std::default_random_engine *generator;
  std::uniform_real_distribution<float> *distribution;

public:
  UniformNoiseGenerator(float l, float h);

  ~UniformNoiseGenerator();

  /**
    @param x unused
  */
  float gen(float x);
};

/**
  Generates noise that oscillates with time.
*/
class SinusoidalNoiseGenerator : public NoiseGenerator {
protected:
  float a;

public:
  /**
    Establishes generator bounds.

    @param l lower bound
    @param h upper bound
    @param wavelength wavelength of oscillation in time units
  */
  SinusoidalNoiseGenerator(float l, float h, float wavelength);

  /**
    @param t current time
  */
  float gen(float t);
};

/**
  A collection of generators that contribute to the same noise output.
*/
class CompoundNoiseGenerator : public NoiseGenerator {
protected:
  std::vector<NoiseGenerator*> generators;

public:
  CompoundNoiseGenerator(float l, float h);

  ~CompoundNoiseGenerator();

  void add_generator(NoiseGenerator *gen);

  float gen(float x);
};

#endif
