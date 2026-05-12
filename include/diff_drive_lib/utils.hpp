#pragma once

#include <cmath>

namespace diff_drive_lib {

inline float rampValue(float current, float target, float dt_s, float accel_limit,
                       float decel_limit) {
  float diff = target - current;
  if (diff == 0.0F) return target;

  // Decelerating = moving towards zero or reducing magnitude
  bool decelerating = (std::abs(target) < std::abs(current)) || (target * current < 0.0F);
  float limit = decelerating ? decel_limit : accel_limit;

  if (limit <= 0.0F) return target;  // No limit

  float max_change = limit * dt_s;
  if (diff > max_change)
    return current + max_change;
  else if (diff < -max_change)
    return current - max_change;
  else
    return target;
}

inline float clamp(const float value, const float limit) {
  if (value > limit)
    return limit;
  else if (value < -limit)
    return -limit;
  else
    return value;
}

}  // namespace diff_drive_lib
