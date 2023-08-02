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

class RobotController {
 public:
  RobotController(const RobotConfiguration& robot_conf);

  /**
   * Initialize the Robot Controller and all Wheel Controllers.
   * @param params Parameter values to use.
   */
  void init(const RobotParams& params);

  /**
   * Update parameters of Robot Controller and all Wheel Controllers
   * @param params Parameter values to use.
   */
  void updateParams(const RobotParams& params);

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
  RobotOdom getOdom();

  /**
   * Get the current wheel states.
   */
  RobotWheelStates getWheelStates();

  /**
   * Reset the odometry position.
   */
  void resetOdom();

  /**
   * Perform an update routine.
   * @param dt_ms Time elapsed since the last call to update function.
   */
  virtual void update(uint32_t dt_ms) = 0;

  /**
   * Enable the controller.
   * Enabling the controller enables all wheel controllers.
   */
  void enable();

  /**
   * Disable the controller.
   * Disabling the controller disables all wheel controllers.
   */
  void disable();

  // Wheel controllers
  WheelController wheel_FL;
  WheelController wheel_RL;
  WheelController wheel_FR;
  WheelController wheel_RR;

 protected:
  RobotOdom odom_;
  bool enabled_ = false;
  int last_command_time_remaining_;
  RobotParams params_;
};

}  // namespace diff_drive_lib