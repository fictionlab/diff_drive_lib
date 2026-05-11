#pragma once

#include <vector>

#include "diff_drive_lib/robot_controller.hpp"
#include "diff_drive_lib/utils.hpp"

namespace diff_drive_lib {

template <size_t VELOCITY_ROLLING_WINDOW_SIZE>
class MecanumController : public RobotController<VELOCITY_ROLLING_WINDOW_SIZE> {
 public:
  using RobotController<VELOCITY_ROLLING_WINDOW_SIZE>::RobotController;

  void setSpeed(float linear_x, float linear_y, float angular) override {
    if (this->params_.robot_input_timeout > 0)
      this->last_command_time_remaining_ = this->params_.robot_input_timeout;
    if (!this->enabled_) this->enable();

    target_linear_x_ = linear_x;
    target_linear_y_ = linear_y;
    target_angular_ = angular;
  }

  void update(const uint32_t dt_ms) override {
    this->checkTimeout(dt_ms);

    const float dt_s = static_cast<float>(dt_ms) * 0.001F;

    if (this->enabled_) rampAndSetWheelVelocities(dt_s);

    this->updateWheels(dt_ms);
    this->updateOdometry(dt_s);
  }

  void disable() override {
    RobotController<VELOCITY_ROLLING_WINDOW_SIZE>::disable();
    target_linear_x_ = 0.0F;
    target_linear_y_ = 0.0F;
    target_angular_ = 0.0F;
    cmd_linear_x_ = 0.0F;
    cmd_linear_y_ = 0.0F;
    cmd_angular_ = 0.0F;
  }

 private:
  void rampAndSetWheelVelocities(float dt_s) {
    cmd_linear_x_ = rampValue(cmd_linear_x_, target_linear_x_, dt_s,
                              this->params_.robot_linear_acceleration,
                              this->params_.robot_linear_deceleration);
    cmd_linear_y_ = rampValue(cmd_linear_y_, target_linear_y_, dt_s,
                              this->params_.robot_linear_acceleration,
                              this->params_.robot_linear_deceleration);
    cmd_angular_ = rampValue(cmd_angular_, target_angular_, dt_s,
                             this->params_.robot_angular_acceleration,
                             this->params_.robot_angular_deceleration);

    const float wheel_geometry =
        (this->params_.robot_wheel_base + this->params_.robot_wheel_separation) / 2.0F;
    const float angular_multiplied =
        cmd_angular_ * wheel_geometry * this->params_.robot_angular_velocity_multiplier;
    const float sum_xy = cmd_linear_x_ + cmd_linear_y_;
    const float diff_xy = cmd_linear_x_ - cmd_linear_y_;

    this->wheel_FL.setTargetVelocity((diff_xy - angular_multiplied) / this->params_.robot_wheel_radius);
    this->wheel_FR.setTargetVelocity((sum_xy + angular_multiplied) / this->params_.robot_wheel_radius);
    this->wheel_RL.setTargetVelocity((sum_xy - angular_multiplied) / this->params_.robot_wheel_radius);
    this->wheel_RR.setTargetVelocity((diff_xy + angular_multiplied) / this->params_.robot_wheel_radius);
  }

  void updateOdometry(float dt_s) {
    const float FL_ang_vel = this->wheel_FL.getVelocity();
    const float RL_ang_vel = this->wheel_RL.getVelocity();
    const float FR_ang_vel = this->wheel_FR.getVelocity();
    const float RR_ang_vel = this->wheel_RR.getVelocity();

    const float wheel_geometry =
        (this->params_.robot_wheel_base + this->params_.robot_wheel_separation) / 2.0F;

    this->odom_.velocity_lin_x = (FL_ang_vel + FR_ang_vel + RL_ang_vel + RR_ang_vel) *
                                 this->params_.robot_wheel_radius / 4.0F;
    this->odom_.velocity_lin_y = (-FL_ang_vel + FR_ang_vel + RL_ang_vel - RR_ang_vel) *
                                 this->params_.robot_wheel_radius / 4.0F;
    this->odom_.velocity_ang =
        (-FL_ang_vel + FR_ang_vel - RL_ang_vel + RR_ang_vel) * this->params_.robot_wheel_radius *
        this->params_.robot_angular_velocity_multiplier / (wheel_geometry * 4.0F);

    const float cos_yaw = std::cos(this->odom_.pose_yaw);
    const float sin_yaw = std::sin(this->odom_.pose_yaw);

    this->odom_.pose_x += (this->odom_.velocity_lin_x * cos_yaw -
                           this->odom_.velocity_lin_y * sin_yaw) * dt_s;
    this->odom_.pose_y += (this->odom_.velocity_lin_x * sin_yaw +
                           this->odom_.velocity_lin_y * cos_yaw) * dt_s;
    this->odom_.pose_yaw += this->odom_.velocity_ang * dt_s;
    this->wrapYaw();
  }
  float target_linear_x_ = 0.0F;
  float target_linear_y_ = 0.0F;
  float target_angular_ = 0.0F;
  float cmd_linear_x_ = 0.0F;
  float cmd_linear_y_ = 0.0F;
  float cmd_angular_ = 0.0F;
};

}  // namespace diff_drive_lib