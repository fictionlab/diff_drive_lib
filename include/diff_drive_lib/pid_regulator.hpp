#pragma once

#include <cstdint>

namespace diff_drive_lib {

inline float clamp(const float value, const float limit) {
  if (value > limit)
    return limit;
  else if (value < -limit)
    return -limit;
  else
    return value;
}

class PIDRegulator {
 public:
  PIDRegulator() {}
  explicit PIDRegulator(float Kp, float Ki, float Kd, float range)
      : Kp_(Kp), Ki_(Ki), Kd_(Kd), range_(range){};

  /**
   * Set the PID coefficients.
   */
  void setCoeffs(float Kp, float Ki, float Kd) {
    Kp_ = Kp;
    Ki_ = Ki;
    Kd_ = Kd;
  }

  /**
   * Set the range of the regulator output.
   */
  void setRange(float range) { range_ = range; }

  /**
   * Reset the regulator to initial state.
   */
  void reset() {
    isum_ = 0;
    last_error_ = 0;
    has_last_error_ = false;
  }

  /**
   * Perform an update routine of the regulator.
   * @param error Current error
   * @param dt_ms Time elapsed since the last call to update function
   * @return Output of the regulator
   */
  float update(float error, uint16_t dt_ms) {
    float cur_err;
    if (has_last_error_) {
      cur_err = error - last_error_;
    } else {
      cur_err = 0;
      has_last_error_ = true;
    }
    last_error_ = error;

    isum_ += Ki_ * error * static_cast<float>(dt_ms);

    // Anti-windup
    isum_ = clamp(isum_, range_);

    float val = Kp_ * error + isum_ + Kd_ * cur_err / static_cast<float>(dt_ms);
    val = clamp(val, range_);

    return val;
  }

 private:
  float Kp_ = 1.0F, Ki_ = 0.0F, Kd_ = 0.0F;
  float range_ = 1000.0;
  float isum_ = 0.0F;
  float last_error_ = 0.0F;
  bool has_last_error_ = false;
};

}  // namespace diff_drive_lib