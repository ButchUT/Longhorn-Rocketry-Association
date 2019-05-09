#include "regression.h"

#include <iostream>
#include <vector>

int main() {
  std::vector<float> x, y;
  std::vector<float> coeffs(2);

  x.push_back(0);
  x.push_back(1);
  x.push_back(2);
  x.push_back(3);

  y.push_back(1.05);
  y.push_back(2.1);
  y.push_back(3.85);
  y.push_back(8.3);

  ExponentialRegressor reg(3);
  reg.fit(x, y, coeffs);

  std::cout << coeffs[0] << " " << coeffs[1] << std::endl;
}
