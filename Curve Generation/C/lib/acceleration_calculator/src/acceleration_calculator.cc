#include "acceleration_calculator.h"

double calculate_acceleration(
  std::pair<double, double> *collected_data,
  size_t collected_data_size,
  double base_mass,
  double velocity,
  double height,
  double start_height,
  double radius,
  double drag_coefficient,
  double time
) {
  assert(collected_data_size >= 0);

  if (collected_data_size > 0 && collected_data != NULL) {
    std::pair<double, double> last_recorded_thrust = collected_data[collected_data_size - 1];

    if (time <= last_recorded_thrust.first) {
      return Interp(
        collected_data,
        collected_data_size,
        time
      );
    }
  }

  double drag = calculate_drag(
    start_height,
    height,
    radius,
    drag_coefficient,
    velocity
  );


  double weight = base_mass * GRAVITY;
  double force = -weight - drag;
  return force / base_mass;
}
