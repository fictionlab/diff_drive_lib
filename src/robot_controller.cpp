#include <cmath>
#include <cstdio>

#include "diff_drive_lib/robot_controller.hpp"

namespace diff_drive_lib {

static constexpr float PI = 3.141592653F;

RobotController::RobotController(const RobotConfiguration& robot_conf)
    : wheel_FL(robot_conf.wheel_FL_conf),
      wheel_RL(robot_conf.wheel_RL_conf),
      wheel_FR(robot_conf.wheel_FR_conf),
      wheel_RR(robot_conf.wheel_RR_conf) {}

void RobotController::init(const RobotParams& params) {
  wheel_FL.init(params);
  wheel_RL.init(params);
  wheel_FR.init(params);
  wheel_RR.init(params);
  params_ = params;
}

void RobotController::updateParams(const RobotParams& params) {
  wheel_FL.updateParams(params);
  wheel_RL.updateParams(params);
  wheel_FR.updateParams(params);
  wheel_RR.updateParams(params);
  params_ = params;
}

void RobotController::enable() {
  wheel_FL.enable();
  wheel_RL.enable();
  wheel_FR.enable();
  wheel_RR.enable();
  enabled_ = true;
}

void RobotController::disable() {
  wheel_FL.disable();
  wheel_RL.disable();
  wheel_FR.disable();
  wheel_RR.disable();
  enabled_ = false;
}

RobotOdom RobotController::getOdom() {
  return odom_;
}

void RobotController::resetOdom() {
  odom_.pose_x = 0.0F;
  odom_.pose_y = 0.0F;
  odom_.pose_yaw = 0.0F;
}

RobotWheelStates RobotController::getWheelStates() {
  RobotWheelStates ws;

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

}  // namespace diff_drive_lib