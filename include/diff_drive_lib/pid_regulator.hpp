#pragma once

#include <cstdint>
#include <cmath>

#include "diff_drive_lib/utils.hpp"

namespace diff_drive_lib {

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
   * @param dt_ms Time elapsed since the last call to update function in milliseconds
   * @return Output of the regulator
   */
  float update(float error, uint32_t dt_ms) {
    if (dt_ms == 0) return 0.0F;

    float err_diff;
    if (has_last_error_) {
      err_diff = error - last_error_;
    } else {
      err_diff = 0;
      has_last_error_ = true;
    }
    last_error_ = error;

    const float dt_s = static_cast<float>(dt_ms) * 0.001F;

    // P term + D term
    float val = Kp_ * error + Kd_ * err_diff / dt_s;

    // I term
    isum_ += Ki_ * error * dt_s;

    // I-term Anti-windup: clamp isum to range to prevent unbounded accumulation
    isum_ = clamp(isum_, range_);

    // Output
    val = clamp(val + isum_, range_);

    return val;
  }

 private:
  float Kp_ = 1.0F, Ki_ = 0.0F, Kd_ = 0.0F;
  float range_ = 30.0F;
  float isum_ = 0.0F;
  float last_error_ = 0.0F;
  bool has_last_error_ = false;
};

}  // namespace diff_drive_lib