#include "constant_area_drag_calculator.h"

#include <cassert>

double calculate_drag(
  double start_height,
  double height,
  double radius,
  double drag_coefficient,
  double velocity
) {
  /*assert(start_height >= 0.0);
  assert(height >= 0.0);
  assert(radius >= 0.0);
  assert(drag_coefficient >= 0.0);
  assert(velocity >= 0.0);*/

  double density = calculate_density(start_height, height);
  double area = _PI * radius * radius;

  return (0.5 * density * velocity * velocity *
    drag_coefficient * area);
}
