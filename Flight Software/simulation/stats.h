#include <vector>

class Dataset {
protected:
  std::vector<float> data;
  float set_mean, set_var, set_stdev;
  bool initial_compute;

  void compute();

public:
  Dataset();

  void add(float f);

  float mean();

  float var();

  float stdev();
};
