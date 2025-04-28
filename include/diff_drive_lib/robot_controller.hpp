#pragma once

#include <vector>

#include "diff_drive_lib/wheel_controller.hpp"

namespace diff_drive_lib {

struct RobotConfiguration {
  WheelConfiguration wheel_FL_conf;
  WheelConfiguration wheel_RL_conf;
  WheelConfiguration wheel_FR_conf;
  WheelConfiguration wheel_RR_conf;
};

struct RobotParams : WheelParams {
  // The radius of the wheel in meters.
  float robot_wheel_radius;

  // The distance (in meters) between the centers of the left and right wheels.
  float robot_wheel_separation;

  // The distance (in meters) between the centers of the rear and front wheels.
  float robot_wheel_base;

  // The angular velocity in setSpeed command is multiplied by this parameter
  // and the calculated odometry has its angular velocity divided by this
  // parameter.
  float robot_angular_velocity_multiplier;

  // The timeout (in milliseconds) for the setSpeed commands. The controller
  // will be disabled if it does not receive a command within the specified
  // time. If set to 0, the timeout is disabled.
  int robot_input_timeout;
};

struct RobotOdom {
  float velocity_lin_x;
  float velocity_lin_y;
  float velocity_ang;
  float pose_x;
  float pose_y;
  float pose_yaw;
};

struct RobotWheelStates {
  float position[4];
  float velocity[4];
  float torque[4];
  float pwm_duty_cycle[4];
};

template <size_t VELOCITY_ROLLING_WINDOW_SIZE>
class RobotController {
 public:
  RobotController(const RobotConfiguration& robot_conf)
      : wheel_FL(robot_conf.wheel_FL_conf),
        wheel_RL(robot_conf.wheel_RL_conf),
        wheel_FR(robot_conf.wheel_FR_conf),
        wheel_RR(robot_conf.wheel_RR_conf){};

  /**
   * Initialize the Robot Controller and all Wheel Controllers.
   * @param params Parameter values to use.
   */
  void init(const RobotParams& params) {
    wheel_FL.init(params);
    wheel_RL.init(params);
    wheel_FR.init(params);
    wheel_RR.init(params);
    params_ = params;
  }

  /**
   * Update parameters of Robot Controller and all Wheel Controllers
   * @param params Parameter values to use.
   */
  void updateParams(const RobotParams& params) {
    wheel_FL.updateParams(params);
    wheel_RL.updateParams(params);
    wheel_FR.updateParams(params);
    wheel_RR.updateParams(params);
    params_ = params;
  }

  /**
   * Set the target speed of the robot.
   * Automatically enables the controller.
   * @param linear_x The linear speed of the robot in x axis in m/s
   * @param linear_y The linear speed of the robot in y axis in m/s
   * @param angular The angular speed of the robot in rad/s
   */
  virtual void setSpeed(float linear_x, float linear_y, float angular) = 0;

  /**
   * Get the current odometry.
   */
  RobotOdom getOdom() { return odom_; }

  /**
   * Get the current wheel states.
   */
  RobotWheelStates getWheelStates() {
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

    ws.pwm_duty_cycle[0] = wheel_FL.getPWMDutyCycle();
    ws.pwm_duty_cycle[1] = wheel_RL.getPWMDutyCycle();
    ws.pwm_duty_cycle[2] = wheel_FR.getPWMDutyCycle();
    ws.pwm_duty_cycle[3] = wheel_RR.getPWMDutyCycle();

    return ws;
  }

  /**
   * Reset the odometry position.
   */
  void resetOdom() {
    odom_.pose_x = 0.0F;
    odom_.pose_y = 0.0F;
    odom_.pose_yaw = 0.0F;
  }

  /**
   * Perform an update routine.
   * @param dt_ms Time elapsed since the last call to update function.
   */
  virtual void update(uint32_t dt_ms) = 0;

  /**
   * Enable the controller.
   * Enabling the controller enables all wheel controllers.
   */
  void enable() {
    wheel_FL.enable();
    wheel_RL.enable();
    wheel_FR.enable();
    wheel_RR.enable();
    enabled_ = true;
  }

  /**
   * Disable the controller.
   * Disabling the controller disables all wheel controllers.
   */
  void disable() {
    wheel_FL.disable();
    wheel_RL.disable();
    wheel_FR.disable();
    wheel_RR.disable();
    enabled_ = false;
  }

  // Wheel controllers
  WheelController<VELOCITY_ROLLING_WINDOW_SIZE> wheel_FL;
  WheelController<VELOCITY_ROLLING_WINDOW_SIZE> wheel_RL;
  WheelController<VELOCITY_ROLLING_WINDOW_SIZE> wheel_FR;
  WheelController<VELOCITY_ROLLING_WINDOW_SIZE> wheel_RR;

 protected:
  RobotOdom odom_;
  bool enabled_ = false;
  int last_command_time_remaining_;
  RobotParams params_;
};

}  // namespace diff_drive_lib