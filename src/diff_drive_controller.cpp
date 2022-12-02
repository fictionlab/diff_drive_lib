#include <cmath>
#include <cstdio>

#include "diff_drive_lib/diff_drive_controller.hpp"

namespace diff_drive_lib {

static constexpr float PI = 3.141592653F;

DiffDriveController::DiffDriveController(const DiffDriveConfiguration& dd_conf)
    : wheel_FL(dd_conf.wheel_FL_conf),
      wheel_RL(dd_conf.wheel_RL_conf),
      wheel_FR(dd_conf.wheel_FR_conf),
      wheel_RR(dd_conf.wheel_RR_conf) {}

void DiffDriveController::init(const DiffDriveParams& params) {
  wheel_FL.init(params);
  wheel_RL.init(params);
  wheel_FR.init(params);
  wheel_RR.init(params);
  params_ = params;
}

void DiffDriveController::updateParams(const DiffDriveParams& params) {
  wheel_FL.updateParams(params);
  wheel_RL.updateParams(params);
  wheel_FR.updateParams(params);
  wheel_RR.updateParams(params);
  params_ = params;
}

void DiffDriveController::enable() {
  wheel_FL.enable();
  wheel_RL.enable();
  wheel_FR.enable();
  wheel_RR.enable();
  enabled_ = true;
}

void DiffDriveController::disable() {
  wheel_FL.disable();
  wheel_RL.disable();
  wheel_FR.disable();
  wheel_RR.disable();
  enabled_ = false;
}

void DiffDriveController::setSpeed(const float linear, const float angular) {
  if (params_.dd_input_timeout > 0)
    last_command_time_remaining_ = params_.dd_input_timeout;
  if (!enabled_) enable();

  const float angular_multiplied =
      angular * params_.dd_angular_velocity_multiplier;
  const float wheel_L_lin_vel =
      linear - (angular_multiplied * params_.dd_wheel_separation / 2.0F);
  const float wheel_R_lin_vel =
      linear + (angular_multiplied * params_.dd_wheel_separation / 2.0F);
  const float wheel_L_ang_vel = wheel_L_lin_vel / params_.dd_wheel_radius;
  const float wheel_R_ang_vel = wheel_R_lin_vel / params_.dd_wheel_radius;

  wheel_FL.setTargetVelocity(wheel_L_ang_vel);
  wheel_RL.setTargetVelocity(wheel_L_ang_vel);
  wheel_FR.setTargetVelocity(wheel_R_ang_vel);
  wheel_RR.setTargetVelocity(wheel_R_ang_vel);
}

DiffDriveOdom DiffDriveController::getOdom() {
  return odom_;
}

void DiffDriveController::resetOdom() {
  odom_.pose_x = 0.0F;
  odom_.pose_y = 0.0F;
  odom_.pose_yaw = 0.0F;
}

DiffDriveWheelStates DiffDriveController::getWheelStates() {
  DiffDriveWheelStates ws;

  ws.position[0] = wheel_FL.getDistance();
  ws.position[1] = wheel_RL.getDistance();
  ws.position[2] = wheel_FR.getDistance();
  ws.position[3] = wheel_RR.getDistance();

  ws.velocity[0] = wheel_FL.getVelocity();
  ws.velocity[1] = wheel_RL.getVelocity();
  ws.velocity[2] = wheel_FR.getVelocity();
  ws.velocity[3] = wheel_RR.getVelocity();

  ws.torque[0] = wheel_FL.getTorque();
  ws.torque[1] = wheel_RL.getTorque();
  ws.torque[2] = wheel_FR.getTorque();
  ws.torque[3] = wheel_RR.getTorque();

  ws.pwm_duty_cycle[0] = wheel_FL.motor.getPWMDutyCycle();
  ws.pwm_duty_cycle[1] = wheel_RL.motor.getPWMDutyCycle();
  ws.pwm_duty_cycle[2] = wheel_FR.motor.getPWMDutyCycle();
  ws.pwm_duty_cycle[3] = wheel_RR.motor.getPWMDutyCycle();

  return ws;
}

void DiffDriveController::update(uint32_t dt_ms) {
  if (enabled_ && params_.dd_input_timeout > 0) {
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

  const float L_ang_vel = (FL_ang_vel + RL_ang_vel) / 2.0F;
  const float R_ang_vel = (FR_ang_vel + RR_ang_vel) / 2.0F;

  // velocity in meters per second
  const float L_lin_vel = L_ang_vel * params_.dd_wheel_radius;
  const float R_lin_vel = R_ang_vel * params_.dd_wheel_radius;

  const float dt_s = static_cast<float>(dt_ms) * 0.001F;

  // linear (m/s) and angular (r/s) velocities of the robot
  odom_.velocity_lin = (L_lin_vel + R_lin_vel) / 2.0F;
  odom_.velocity_ang = (R_lin_vel - L_lin_vel) / params_.dd_wheel_separation;

  odom_.velocity_ang /= params_.dd_angular_velocity_multiplier;

  // Integrate the velocity using the rectangle rule
  odom_.pose_yaw += odom_.velocity_ang * dt_s;
  if (odom_.pose_yaw > 2.0F * PI)
    odom_.pose_yaw -= 2.0F * PI;
  else if (odom_.pose_yaw < 0.0F)
    odom_.pose_yaw += 2.0F * PI;

  odom_.pose_x += odom_.velocity_lin * std::cos(odom_.pose_yaw) * dt_s;
  odom_.pose_y += odom_.velocity_lin * std::sin(odom_.pose_yaw) * dt_s;
}

}  // namespace diff_drive_lib