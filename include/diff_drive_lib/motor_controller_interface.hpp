#pragma once

#include <stdint.h>

#include "diff_drive_lib/utils.hpp"

namespace diff_drive_lib {

struct MotorControllerInterface {
  /**
   * Initialize the Motor Controller.
   */
  virtual void init() = 0;

  /**
   * Set the supply voltage (voltage applied to the motor at 100% PWM).
   * @param voltage Supply voltage in Volts
   */
  virtual void setSupplyVoltage(float voltage) = 0;

  /**
   * Get the supply voltage (voltage applied to the motor at 100% PWM).
   * @return Supply voltage in Volts
   */
  virtual float getSupplyVoltage() = 0;

  /**
   * Set the desired motor voltage. The motor controller converts this
   * to a PWM duty cycle based on the current supply voltage.
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
  void setSupplyVoltage(float voltage) { supply_voltage_ = voltage; }
  float getSupplyVoltage() override { return supply_voltage_; }

  void setVoltage(float voltage) {
    if (supply_voltage_ > 0.0F) {
      const float clamped_voltage = clamp(voltage, supply_voltage_);
      setPWMDutyCycle((clamped_voltage / supply_voltage_) * 100.0F);
    } else {
      setPWMDutyCycle(0.0F);
    }
  }

 protected:
  float supply_voltage_ = 12.0F;
};

}  // namespace diff_drive_lib