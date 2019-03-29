#include <chrono>
#include <math.h>
#include "noise.h"

NoiseGenerator::NoiseGenerator(float l, float h) {
  low = l;
  high = h;
}

UniformNoiseGenerator::UniformNoiseGenerator(float l, float h) :
  NoiseGenerator(l, h) {
  unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  generator = new std::default_random_engine(seed);
  distribution = new std::uniform_real_distribution<float>(l, h);
}

UniformNoiseGenerator::~UniformNoiseGenerator() {
  delete generator;
  delete distribution;
}

float UniformNoiseGenerator::gen(float x) {
  return (*distribution)(*generator);
}

SinusoidalNoiseGenerator::SinusoidalNoiseGenerator(float l, float h,
  float wavelength) : NoiseGenerator(l, h) {
  a = (2 * M_PI) / wavelength;
}

float SinusoidalNoiseGenerator::gen(float t) {
  return sin(a * t) * (high - low) * 0.5 + (low + (high - low) * 0.5);
}

CompoundNoiseGenerator::CompoundNoiseGenerator(float l, float h) :
  NoiseGenerator(l, h) {}

CompoundNoiseGenerator::~CompoundNoiseGenerator() {
  for (int i = 0; i < generators.size(); i++)
    free(generators[i]);
}

void CompoundNoiseGenerator::add_generator(NoiseGenerator *gen) {
  generators.push_back(gen);
}

float CompoundNoiseGenerator::gen(float x) {
  float noise = 0;

  for (int i = 0; i < generators.size(); i++)
    noise += (*generators[i]).gen(x);

  return noise < low ? low : (noise > high ? high : noise);
}
