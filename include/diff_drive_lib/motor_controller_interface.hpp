#pragma once

#include <stdint.h>

namespace diff_drive_lib {

struct MotorControllerInterface {
  /**
   * Initialize the Motor Controller.
   */
  virtual void init() = 0;

  /**
   * Set the PWM Duty Cycle to the motor driver
   * @param pwm_duty The PWM Duty Cycle in percents
   */
  virtual void setPWMDutyCycle(float pwm_duty) = 0;

  /**
   * Get the current PWM Duty Cycle
   */
  virtual float getPWMDutyCycle() = 0;

  /**
   * Get the number of encoder ticks.
   * @return encoder ticks
   */
  virtual int32_t getEncoderCnt() = 0;

  /**
   * Set the number of encoder ticks to 0.
   */
  virtual void resetEncoderCnt() = 0;

  /**
   * Get motor winding current in Amperes
   */
  virtual float getWindingCurrent() = 0;
};

}  // namespace diff_drive_lib