#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <utility>

#include "diff_drive_lib/circular_buffer.hpp"
#include "diff_drive_lib/motor_controller_interface.hpp"
#include "diff_drive_lib/pid_regulator.hpp"

namespace diff_drive_lib {

enum class WheelOperationMode {
  VELOCITY,
  POSITION
};

struct WheelConfiguration {
  // The instance of the motor controller.
  MotorControllerInterface& motor;

  // The operation mode to use.
  WheelOperationMode op_mode;

  // The number of velocity samples to average together to compute wheel
  // velocity.
  size_t velocity_rolling_window_size;
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

  // Whether to detect rapid change in encoder tick count.
  bool wheel_encoder_jump_detection_enabled = false;

  // The instantaneous velocity in ticks per second over which the encoder
  // position is assumed to have jumped.
  float wheel_encoder_jump_threshold = 0.0;
};

class WheelController {
 public:
  WheelController(const WheelConfiguration& wheel_conf);

  /**
   * Initialize the Wheel Controller.
   * Should be called after all ROS parameters are loaded.
   * Sets the PID regulator parameters and initializes the Motor Controller.
   */
  void init(const WheelParams& params);

  /**
   * Update parameters of the Wheel Controller
   * @param params Parameter values to use.
   */
  void updateParams(const WheelParams& params);

  /**
   * Perform an update routine.
   * @param dt_ms Time elapsed since the last call to update function
   */
  void update(uint32_t dt_ms);

  /**
   * Set the target velocity of the wheel in rad/s.
   */
  void setTargetVelocity(float speed);

  /**
   * Set the target position of the wheel in rad.
   */
  void setTargetPosition(float position);

  /**
   * Get the current velocity of the motor in rad/s.
   */
  float getVelocity();

  /**
   * Get the current PWM Duty cycle applied to the motor.
   */
  float getPWMDutyCycle();

  /**
   * Get the current output torque of the motor.
   */
  float getTorque();

  /**
   * Get the current distance traversed by the wheel in radians.
   */
  float getDistance();

  /**
   * Reset the distance traversed by the wheel.
   */
  void resetDistance();

  /**
   * Enable the controller.
   * Starts sending PWM duty commands to the motor controller.
   */
  void enable();

  /**
   * Disable the controller.
   * Disabling the wheel controller stops sending PWM duty commands to the motor
   * controller.
   */
  void disable();

  MotorControllerInterface& motor;

 private:
  PIDRegulator pid_reg_;
  WheelOperationMode op_mode_;
  CircularBuffer<std::pair<int32_t, uint32_t>> encoder_buffer_;

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
};

}  // namespace diff_drive_lib