#include <algorithm>

#include "diff_drive_lib/motor_controller_interface.hpp"
#include "diff_drive_lib/wheel_controller.hpp"

namespace diff_drive_lib {

static constexpr float PI = 3.141592653F;

WheelController::WheelController(const WheelConfiguration& wheel_conf)
    : motor(wheel_conf.motor),
      encoder_buffer_(wheel_conf.velocity_rolling_window_size) {}

void WheelController::init(const WheelParams& params) {
  updateParams(params);
  motor.init();
  motor.resetEncoderCnt();
}

void WheelController::updateParams(const WheelParams& params) {
  v_reg_.setCoeffs(params.wheel_pid_p, params.wheel_pid_i, params.wheel_pid_d);
  v_reg_.setRange(std::min(1000.0F, params.wheel_pwm_duty_limit * 10.0F));
  params_ = params;
}

void WheelController::update(const uint32_t dt_ms) {
  int32_t ticks_prev = ticks_now_;
  ticks_now_ = motor.getEncoderCnt() - ticks_offset_;

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

  if (enabled_) {
    float pwm_duty;
    if (v_now_ == 0.0F && v_target_ == 0.0F) {
      v_reg_.reset();
      pwm_duty = 0.0F;
    } else {
      float v_err = v_target_ - v_now_;
      pwm_duty = v_reg_.update(v_err, dt_ms) / 10.0F;
    }
    motor.setPWMDutyCycle(pwm_duty);
  }
}

void WheelController::setTargetVelocity(const float speed) {
  v_target_ = (speed / (2.0F * PI)) * params_.wheel_encoder_resolution;
}

float WheelController::getVelocity() {
  return (v_now_ / params_.wheel_encoder_resolution) * (2.0F * PI);
}

float WheelController::getTorque() {
  return motor.getWindingCurrent() * params_.wheel_torque_constant;
}

float WheelController::getDistance() {
  return (ticks_now_ / params_.wheel_encoder_resolution) * (2.0F * PI);
}

void WheelController::resetDistance() {
  motor.resetEncoderCnt();
  ticks_now_ = 0;
  ticks_offset_ = 0;
}

void WheelController::enable() {
  if (!enabled_) {
    v_reg_.reset();
    enabled_ = true;
  }
}

void WheelController::disable() {
  if (enabled_) {
    enabled_ = false;
    motor.setPWMDutyCycle(0.0F);
  }
}

}  // namespace diff_drive_lib
