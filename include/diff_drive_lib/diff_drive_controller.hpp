#pragma once

#include <vector>

#include "diff_drive_lib/robot_controller.hpp"
#include "diff_drive_lib/utils.hpp"

namespace diff_drive_lib {

template <size_t VELOCITY_ROLLING_WINDOW_SIZE>
class DiffDriveController : public RobotController<VELOCITY_ROLLING_WINDOW_SIZE> {
 public:
  using RobotController<VELOCITY_ROLLING_WINDOW_SIZE>::RobotController;

  void setSpeed(float linear_x, float /*linear_y*/, float angular) override {
    if (this->params_.robot_input_timeout > 0)
      this->last_command_time_remaining_ = this->params_.robot_input_timeout;
    if (!this->enabled_) this->enable();

    target_linear_x_ = linear_x;
    target_angular_ = angular;
  }

  void update(const uint32_t dt_ms) override {
    if (dt_ms == 0) return;

    this->checkTimeout(dt_ms);

    const float dt_s = static_cast<float>(dt_ms) * 0.001F;

    if (this->enabled_) rampAndSetWheelVelocities(dt_s);

    this->updateWheels(dt_ms);
    this->updateOdometry(dt_s);
  }

  void disable() override {
    RobotController<VELOCITY_ROLLING_WINDOW_SIZE>::disable();
    target_linear_x_ = 0.0F;
    target_angular_ = 0.0F;
    cmd_linear_x_ = 0.0F;
    cmd_angular_ = 0.0F;
  }

 private:
  void rampAndSetWheelVelocities(float dt_s) {
    cmd_linear_x_ = rampValue(cmd_linear_x_, target_linear_x_, dt_s,
                              this->params_.robot_linear_acceleration,
                              this->params_.robot_linear_deceleration);
    cmd_angular_ = rampValue(cmd_angular_, target_angular_, dt_s,
                             this->params_.robot_angular_acceleration,
                             this->params_.robot_angular_deceleration);

    const float angular_multiplied = cmd_angular_ * this->params_.robot_angular_velocity_multiplier;
    const float wheel_L_lin_vel =
        cmd_linear_x_ - (angular_multiplied * this->params_.robot_wheel_separation / 2.0F);
    const float wheel_R_lin_vel =
        cmd_linear_x_ + (angular_multiplied * this->params_.robot_wheel_separation / 2.0F);
    const float wheel_L_ang_vel = wheel_L_lin_vel / this->params_.robot_wheel_radius;
    const float wheel_R_ang_vel = wheel_R_lin_vel / this->params_.robot_wheel_radius;

    this->wheel_FL.setTargetVelocity(wheel_L_ang_vel);
    this->wheel_RL.setTargetVelocity(wheel_L_ang_vel);
    this->wheel_FR.setTargetVelocity(wheel_R_ang_vel);
    this->wheel_RR.setTargetVelocity(wheel_R_ang_vel);
  }

  void updateOdometry(float dt_s) {
    const float FL_ang_vel = this->wheel_FL.getVelocity();
    const float RL_ang_vel = this->wheel_RL.getVelocity();
    const float FR_ang_vel = this->wheel_FR.getVelocity();
    const float RR_ang_vel = this->wheel_RR.getVelocity();

    const float L_ang_vel = (FL_ang_vel + RL_ang_vel) / 2.0F;
    const float R_ang_vel = (FR_ang_vel + RR_ang_vel) / 2.0F;

    const float L_lin_vel = L_ang_vel * this->params_.robot_wheel_radius;
    const float R_lin_vel = R_ang_vel * this->params_.robot_wheel_radius;

    this->odom_.velocity_lin_x = (L_lin_vel + R_lin_vel) / 2.0F;
    this->odom_.velocity_ang =
        (R_lin_vel - L_lin_vel) / this->params_.robot_wheel_separation /
        this->params_.robot_angular_velocity_multiplier;

    this->odom_.pose_yaw += this->odom_.velocity_ang * dt_s;
    this->wrapYaw();

    this->odom_.pose_x += this->odom_.velocity_lin_x * std::cos(this->odom_.pose_yaw) * dt_s;
    this->odom_.pose_y += this->odom_.velocity_lin_x * std::sin(this->odom_.pose_yaw) * dt_s;
  }
  float target_linear_x_ = 0.0F;
  float target_angular_ = 0.0F;
  float cmd_linear_x_ = 0.0F;
  float cmd_angular_ = 0.0F;
};

}  // namespace diff_drive_lib