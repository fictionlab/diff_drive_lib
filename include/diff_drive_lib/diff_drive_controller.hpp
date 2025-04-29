#pragma once

#include <vector>

#include "diff_drive_lib/robot_controller.hpp"

namespace diff_drive_lib {

template <size_t VELOCITY_ROLLING_WINDOW_SIZE>
class DiffDriveController : public RobotController<VELOCITY_ROLLING_WINDOW_SIZE> {
 public:
  using RobotController<VELOCITY_ROLLING_WINDOW_SIZE>::RobotController;

  void setSpeed(float linear_x, float /*linear_y*/, float angular) override {
    if (this->params_.robot_input_timeout > 0)
      this->last_command_time_remaining_ = this->params_.robot_input_timeout;
    if (!this->enabled_) this->enable();

    const float angular_multiplied = angular * this->params_.robot_angular_velocity_multiplier;
    const float wheel_L_lin_vel =
        linear_x - (angular_multiplied * this->params_.robot_wheel_separation / 2.0F);
    const float wheel_R_lin_vel =
        linear_x + (angular_multiplied * this->params_.robot_wheel_separation / 2.0F);
    const float wheel_L_ang_vel = wheel_L_lin_vel / this->params_.robot_wheel_radius;
    const float wheel_R_ang_vel = wheel_R_lin_vel / this->params_.robot_wheel_radius;

    this->wheel_FL.setTargetVelocity(wheel_L_ang_vel);
    this->wheel_RL.setTargetVelocity(wheel_L_ang_vel);
    this->wheel_FR.setTargetVelocity(wheel_R_ang_vel);
    this->wheel_RR.setTargetVelocity(wheel_R_ang_vel);
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

    const float L_ang_vel = (FL_ang_vel + RL_ang_vel) / 2.0F;
    const float R_ang_vel = (FR_ang_vel + RR_ang_vel) / 2.0F;

    // velocity in meters per second
    const float L_lin_vel = L_ang_vel * this->params_.robot_wheel_radius;
    const float R_lin_vel = R_ang_vel * this->params_.robot_wheel_radius;

    const float dt_s = static_cast<float>(dt_ms) * 0.001F;

    // linear (m/s) and angular (r/s) velocities of the robot
    this->odom_.velocity_lin_x = (L_lin_vel + R_lin_vel) / 2.0F;
    this->odom_.velocity_ang = (R_lin_vel - L_lin_vel) / this->params_.robot_wheel_separation;

    this->odom_.velocity_ang /= this->params_.robot_angular_velocity_multiplier;

    // Integrate the velocity using the rectangle rule
    this->odom_.pose_yaw += this->odom_.velocity_ang * dt_s;
    if (this->odom_.pose_yaw > 2.0F * PI)
      this->odom_.pose_yaw -= 2.0F * PI;
    else if (this->odom_.pose_yaw < 0.0F)
      this->odom_.pose_yaw += 2.0F * PI;

    this->odom_.pose_x += this->odom_.velocity_lin_x * std::cos(this->odom_.pose_yaw) * dt_s;
    this->odom_.pose_y += this->odom_.velocity_lin_x * std::sin(this->odom_.pose_yaw) * dt_s;
  }

 private:
  static constexpr float PI = 3.141592653F;
};

}  // namespace diff_drive_lib