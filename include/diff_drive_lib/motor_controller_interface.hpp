#pragma once

#include <stdint.h>

namespace diff_drive_lib {

struct MotorControllerInterface {
  /**
   * Initialize the Motor Controller.
   */
  virtual void init() = 0;

  /**
   * Set the battery voltage used for voltage-to-PWM conversion.
   * @param voltage Battery voltage in Volts
   */
  virtual void setBatteryVoltage(float voltage) = 0;

  /**
   * Set the desired motor voltage. The motor controller converts this
   * to a PWM duty cycle based on the current battery voltage.
   * @param voltage Desired motor voltage in Volts
   */
  virtual void setVoltage(float voltage) = 0;

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

struct MotorControllerBase : public MotorControllerInterface {
  void setBatteryVoltage(float voltage) { battery_voltage_ = voltage; }

  void setVoltage(float voltage) {
    if (battery_voltage_ > 0.0F) {
      setPWMDutyCycle((voltage / battery_voltage_) * 100.0F);
    } else {
      setPWMDutyCycle(0.0F);
    }
  }

 protected:
  float battery_voltage_ = 0.0F;
};

}  // namespace diff_drive_lib