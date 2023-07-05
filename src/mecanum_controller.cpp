#include <cmath>
#include <cstdio>

#include "diff_drive_lib/mecanum_controller.hpp"

namespace diff_drive_lib {

static constexpr float PI = 3.141592653F;

void MecanumController::setSpeed(const float linear_x, const float linear_y, const float angular) {
  if (params_.robot_input_timeout > 0)
    last_command_time_remaining_ = params_.robot_input_timeout;
  if (!enabled_) enable();

  const float wheel_geometry = (params_.robot_wheel_base + params_.robot_wheel_separation) / 2.0F;

  const float angular_multiplied = angular * wheel_geometry;
  const float sum_xy = linear_x + linear_y;
  const float diff_xy = linear_x - linear_y;
  
  const float wheel_FL_vel = (diff_xy - angular_multiplied) / params_.robot_wheel_radius;
  const float wheel_FR_vel = (sum_xy + angular_multiplied) / params_.robot_wheel_radius;
  const float wheel_RL_vel = (sum_xy - angular_multiplied) / params_.robot_wheel_radius;
  const float wheel_RR_vel = (diff_xy + angular_multiplied) / params_.robot_wheel_radius;

  wheel_FL.setTargetVelocity(wheel_FL_vel);
  wheel_RL.setTargetVelocity(wheel_RL_vel);
  wheel_FR.setTargetVelocity(wheel_FR_vel);
  wheel_RR.setTargetVelocity(wheel_RR_vel);
}

void MecanumController::update(uint32_t dt_ms) {
  if (enabled_ && params_.robot_input_timeout > 0) {
    last_command_time_remaining_ -= dt_ms;
    if (last_command_time_remaining_ < 0) disable();
  }

  wheel_FL.update(dt_ms);
  wheel_RL.update(dt_ms);
  wheel_FR.update(dt_ms);
  wheel_RR.update(dt_ms);

  // velocity in radians per second
  const float FL_ang_vel = wheel_FL.getVelocity();
  const float RL_ang_vel = wheel_RL.getVelocity();
  const float FR_ang_vel = wheel_FR.getVelocity();
  const float RR_ang_vel = wheel_RR.getVelocity();

  const float dt_s = static_cast<float>(dt_ms) * 0.001F;

  const float wheel_geometry = (params_.robot_wheel_base + params_.robot_wheel_separation) / 2.0F;

  const float velocity_lin_x = (FL_ang_vel+FR_ang_vel+RL_ang_vel+RR_ang_vel) * params_.robot_wheel_radius / 4.0 ;
  const float velocity_lin_y = (-FL_ang_vel+FR_ang_vel+RL_ang_vel-RR_ang_vel) * params_.robot_wheel_radius / 4.0;
  const float z_rotation = (-FL_ang_vel+FR_ang_vel-RL_ang_vel+RR_ang_vel) * params_.robot_wheel_radius * params_.robot_angular_velocity_multiplier / (wheel_geometry * 4.0F);

  const float x_move = velocity_lin_x * std::cos(odom_.pose_yaw) - velocity_lin_y * std::sin(odom_.pose_yaw);
  const float y_move = velocity_lin_x * std::sin(odom_.pose_yaw) + velocity_lin_y * std::cos(odom_.pose_yaw);

  odom_.pose_x += (x_move * dt_s);
  odom_.pose_y += (y_move * dt_s);

  odom_.pose_yaw += (z_rotation * dt_s);

  if (odom_.pose_yaw > 2.0F * PI)
    odom_.pose_yaw -= 2.0F * PI;
  else if (odom_.pose_yaw < 0.0F)
    odom_.pose_yaw += 2.0F * PI;

  odom_.velocity_ang = z_rotation;
  odom_.velocity_lin_x = velocity_lin_x;
  odom_.velocity_lin_y = velocity_lin_y;
}

}  // namespace diff_drive_lib