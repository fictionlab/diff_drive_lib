#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <utility>

#include "diff_drive_lib/circular_buffer.hpp"
#include "diff_drive_lib/motor_controller_interface.hpp"
#include "diff_drive_lib/pid_regulator.hpp"

namespace diff_drive_lib {

enum class WheelOperationMode { VELOCITY, POSITION };

struct WheelConfiguration {
  // The instance of the motor controller.
  MotorControllerInterface& motor;

  // The operation mode to use.
  WheelOperationMode op_mode;

  // Whether to reverse the direction of the wheel.
  bool reversed = false;
};

struct WheelParams {
  // The resolution of the wheel encoder in counts per rotation.
  float wheel_encoder_resolution = 100.0;

  // The torque in Nm produced by the wheel per 1 Ampere of windind current.
  float wheel_torque_constant = 1.0;

  // P constant of the PID regulator.
  float wheel_pid_p = 1.0;

  // I constant of the PID regulator.
  float wheel_pid_i = 0.0;

  // D constant of the PID regulator.
  float wheel_pid_d = 0.0;

  // The limit of the PWM duty applied to the motor in percent.
  float wheel_pwm_duty_limit = 100.0;

  // The maximum rate of change in pwm duty cycle per millisecond
  float wheel_pwm_duty_ramp = 10.0;

  // Whether to detect rapid change in encoder tick count.
  bool wheel_encoder_jump_detection_enabled = false;

  // The instantaneous velocity in ticks per second over which the encoder
  // position is assumed to have jumped.
  float wheel_encoder_jump_threshold = 0.0;
};

template <size_t VELOCITY_ROLLING_WINDOW_SIZE>
class WheelController {
 public:
  WheelController(const WheelConfiguration& wheel_conf)
      : motor(wheel_conf.motor), op_mode_(wheel_conf.op_mode), reversed_(wheel_conf.reversed) {};

  /**
   * Initialize the Wheel Controller.
   * Should be called after all ROS parameters are loaded.
   * Sets the PID regulator parameters and initializes the Motor Controller.
   */
  void init(const WheelParams& params) {
    updateParams(params);
    motor.init();
    motor.resetEncoderCnt();
  }

  /**
   * Update parameters of the Wheel Controller
   * @param params Parameter values to use.
   */
  void updateParams(const WheelParams& params) {
    pid_reg_.setCoeffs(params.wheel_pid_p, params.wheel_pid_i, params.wheel_pid_d);
    pid_reg_.setRange(std::min(1000.0F, params.wheel_pwm_duty_limit * 10.0F));
    params_ = params;
  }

  /**
   * Perform an update routine.
   * @param dt_ms Time elapsed since the last call to update function
   */
  void update(uint32_t dt_ms) {
    int32_t ticks_prev = ticks_now_;
    ticks_now_ = this->getEncoderCnt();

    int32_t new_ticks = ticks_now_ - ticks_prev;

    if (params_.wheel_encoder_jump_detection_enabled) {
      float ins_vel = static_cast<float>(std::abs(new_ticks)) / (dt_ms * 0.001F);
      if (ins_vel > params_.wheel_encoder_jump_threshold) {
        ticks_offset_ += new_ticks;
        ticks_now_ -= new_ticks;
        new_ticks = 0;
      }
    }

    std::pair<int32_t, uint32_t> encoder_old =
        encoder_buffer_.push_back(std::pair<int32_t, uint32_t>(new_ticks, dt_ms));

    ticks_sum_ += new_ticks;
    dt_sum_ += dt_ms;

    ticks_sum_ -= encoder_old.first;
    dt_sum_ -= encoder_old.second;

    v_now_ = static_cast<float>(ticks_sum_) / (dt_sum_ * 0.001F);

    float target_pwm_duty = 0.0F;
    if (enabled_) {
      if (op_mode_ == WheelOperationMode::VELOCITY) {
        if (v_now_ == 0.0F && v_target_ == 0.0F) {
          pid_reg_.reset();
          target_pwm_duty = 0.0F;
        } else {
          float v_err = v_target_ - v_now_;
          target_pwm_duty = pid_reg_.update(v_err, dt_ms) / 10.0F;
        }
      } else if (op_mode_ == WheelOperationMode::POSITION) {
        float ticks_err = ticks_target_ - ticks_now_;
        target_pwm_duty = pid_reg_.update(ticks_err, dt_ms) / 10.0F;
      }

      float pwm_duty;
      float current_pwm_duty = this->getPWMDutyCycle();
      float max_pwm_change = params_.wheel_pwm_duty_ramp * static_cast<float>(dt_ms);
      if (target_pwm_duty > current_pwm_duty) {
        pwm_duty = std::min(target_pwm_duty, current_pwm_duty + max_pwm_change);
      } else {
        pwm_duty = std::max(target_pwm_duty, current_pwm_duty - max_pwm_change);
      }

      this->setPWMDutyCycle(pwm_duty);
    }
  }

  /**
   * Set the target velocity of the wheel in rad/s.
   * @param speed The target speed in rad/s.
   */
  void setTargetVelocity(float speed) {
    v_target_ = (speed / (2.0F * PI)) * params_.wheel_encoder_resolution;
  }

  /**
   * Set the target position of the wheel in rad.
   * @param position The target position in radians.
   */
  void setTargetPosition(float position) {
    ticks_target_ = (position / (2.0F * PI)) * params_.wheel_encoder_resolution;
  }

  /**
   * Set the PWM Duty Cycle to the motor controller.
   * @param pwm_duty The PWM Duty Cycle in percents
   */
  void setPWMDutyCycle(float pwm_duty) {
    if (reversed_) {
      motor.setPWMDutyCycle(-pwm_duty);
    } else {
      motor.setPWMDutyCycle(pwm_duty);
    }
  }

  /**
   * Get the current velocity of the motor in rad/s.
   */
  float getVelocity() { return (v_now_ / params_.wheel_encoder_resolution) * (2.0F * PI); }

  /**
   * Get the current PWM Duty cycle applied to the motor.
   */
  float getPWMDutyCycle() {
    float duty = motor.getPWMDutyCycle();
    return reversed_ ? -duty : duty;
  }

  /**
   * Get the current encoder count of the motor.
   * @return The current encoder count.
   */
  int32_t getEncoderCnt() {
    int32_t raw_encoder = motor.getEncoderCnt();
    return (reversed_ ? -raw_encoder : raw_encoder) + ticks_offset_;
  }

  /**
   * Get the current output torque of the motor.
   */
  float getTorque() { return motor.getWindingCurrent() * params_.wheel_torque_constant; }

  /**
   * Get the current distance traversed by the wheel in radians.
   */
  float getDistance() { return (ticks_now_ / params_.wheel_encoder_resolution) * (2.0F * PI); }

  /**
   * Reset the distance traversed by the wheel.
   * @param position The value to reset distance to (default 0.0).
   */
  void resetDistance(float position = 0.0F) {
    motor.resetEncoderCnt();
    ticks_now_ = 0;
    ticks_offset_ = -(position / (2.0F * PI)) * params_.wheel_encoder_resolution;
  }

  /**
   * Enable the controller.
   * Starts sending PWM duty commands to the motor controller.
   */
  void enable() {
    if (!enabled_) {
      pid_reg_.reset();
      enabled_ = true;
    }
  }

  /**
   * Disable the controller.
   * Disabling the wheel controller stops sending PWM duty commands to the motor
   * controller.
   */
  void disable() {
    if (enabled_) {
      enabled_ = false;
      motor.setPWMDutyCycle(0.0F);
    }
  }

  MotorControllerInterface& motor;

 private:
  PIDRegulator pid_reg_;
  WheelOperationMode op_mode_;
  bool reversed_;
  CircularBuffer<std::pair<int32_t, uint32_t>, VELOCITY_ROLLING_WINDOW_SIZE> encoder_buffer_;

  int16_t power_ = 0;

  bool enabled_ = false;

  int32_t ticks_now_ = 0;
  int32_t ticks_sum_ = 0;
  int32_t ticks_offset_ = 0;
  uint32_t dt_sum_ = 0;

  float v_now_ = 0.0F;

  // Velocity mode
  float v_target_ = 0.0F;

  // Position mode
  float ticks_target_ = 0.0F;

  WheelParams params_;

  static constexpr float PI = 3.141592653F;
};

}  // namespace diff_drive_lib