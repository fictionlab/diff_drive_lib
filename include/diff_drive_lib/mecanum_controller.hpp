#pragma once

#include <vector>

#include "diff_drive_lib/robot_controller.hpp"

namespace diff_drive_lib {

template <size_t VELOCITY_ROLLING_WINDOW_SIZE>
class MecanumController : public RobotController<VELOCITY_ROLLING_WINDOW_SIZE> {
 public:
  using RobotController<VELOCITY_ROLLING_WINDOW_SIZE>::RobotController;

  void setSpeed(float linear_x, float linear_y, float angular) override {
    if (this->params_.robot_input_timeout > 0)
      this->last_command_time_remaining_ = this->params_.robot_input_timeout;
    if (!this->enabled_) this->enable();

    const float wheel_geometry =
        (this->params_.robot_wheel_base + this->params_.robot_wheel_separation) / 2.0F;

    const float angular_multiplied =
        angular * wheel_geometry * this->params_.robot_angular_velocity_multiplier;
    const float sum_xy = linear_x + linear_y;
    const float diff_xy = linear_x - linear_y;

    const float wheel_FL_vel = (diff_xy - angular_multiplied) / this->params_.robot_wheel_radius;
    const float wheel_FR_vel = (sum_xy + angular_multiplied) / this->params_.robot_wheel_radius;
    const float wheel_RL_vel = (sum_xy - angular_multiplied) / this->params_.robot_wheel_radius;
    const float wheel_RR_vel = (diff_xy + angular_multiplied) / this->params_.robot_wheel_radius;

    this->wheel_FL.setTargetVelocity(wheel_FL_vel);
    this->wheel_RL.setTargetVelocity(wheel_RL_vel);
    this->wheel_FR.setTargetVelocity(wheel_FR_vel);
    this->wheel_RR.setTargetVelocity(wheel_RR_vel);
  }

  void update(uint32_t dt_ms) override {
    if (this->enabled_ && this->params_.robot_input_timeout > 0) {
      this->last_command_time_remaining_ -= dt_ms;
      if (this->last_command_time_remaining_ < 0) this->disable();
    }

    this->wheel_FL.update(dt_ms);
    this->wheel_RL.update(dt_ms);
    this->wheel_FR.update(dt_ms);
    this->wheel_RR.update(dt_ms);

    // velocity in radians per second
    const float FL_ang_vel = this->wheel_FL.getVelocity();
    const float RL_ang_vel = this->wheel_RL.getVelocity();
    const float FR_ang_vel = this->wheel_FR.getVelocity();
    const float RR_ang_vel = this->wheel_RR.getVelocity();

    const float dt_s = static_cast<float>(dt_ms) * 0.001F;

    const float wheel_geometry =
        (this->params_.robot_wheel_base + this->params_.robot_wheel_separation) / 2.0F;

    const float velocity_lin_x = (FL_ang_vel + FR_ang_vel + RL_ang_vel + RR_ang_vel) *
                                 this->params_.robot_wheel_radius / 4.0F;
    const float velocity_lin_y = (-FL_ang_vel + FR_ang_vel + RL_ang_vel - RR_ang_vel) *
                                 this->params_.robot_wheel_radius / 4.0F;
    const float z_rotation =
        (-FL_ang_vel + FR_ang_vel - RL_ang_vel + RR_ang_vel) * this->params_.robot_wheel_radius *
        this->params_.robot_angular_velocity_multiplier / (wheel_geometry * 4.0F);

    const float x_move = velocity_lin_x * std::cos(this->odom_.pose_yaw) -
                         velocity_lin_y * std::sin(this->odom_.pose_yaw);
    const float y_move = velocity_lin_x * std::sin(this->odom_.pose_yaw) +
                         velocity_lin_y * std::cos(this->odom_.pose_yaw);

    this->odom_.pose_x += (x_move * dt_s);
    this->odom_.pose_y += (y_move * dt_s);

    this->odom_.pose_yaw += (z_rotation * dt_s);

    if (this->odom_.pose_yaw > 2.0F * PI)
      this->odom_.pose_yaw -= 2.0F * PI;
    else if (this->odom_.pose_yaw < 0.0F)
      this->odom_.pose_yaw += 2.0F * PI;

    this->odom_.velocity_ang = z_rotation;
    this->odom_.velocity_lin_x = velocity_lin_x;
    this->odom_.velocity_lin_y = velocity_lin_y;
  }

 private:
  static constexpr float PI = 3.141592653F;
};

}  // namespace diff_drive_lib