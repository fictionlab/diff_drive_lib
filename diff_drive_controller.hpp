#pragma once

#include <vector>

#include "wheel_controller.hpp"

struct DiffDriveConfiguration {
  WheelConfiguration wheel_FL_conf;
  WheelConfiguration wheel_RL_conf;
  WheelConfiguration wheel_FR_conf;
  WheelConfiguration wheel_RR_conf;
};

struct DiffDriveParams : WheelParams {
  float dd_wheel_radius;
  float dd_wheel_separation;
  float dd_angular_velocity_multiplier;
  int dd_input_timeout;
};

struct DiffDriveOdom {
  float velocity_lin;
  float velocity_ang;
  float pose_x;
  float pose_y;
  float pose_yaw;
};

struct DiffDriveWheelStates {
  float position[4];
  float velocity[4];
  float torque[4];
  float pwm_duty_cycle[4];
};

class DiffDriveController {
 public:
  DiffDriveController(const DiffDriveConfiguration& dd_conf);

  /**
   * Initialize the Diff Drive Controller and all Wheel Controllers.
   * @param params Parameter values to use.
   */
  void init(const DiffDriveParams& params);

  /**
   * Set the target speed of the robot.
   * Automatically enables the controller.
   * @param linear The linear speed of the robot in m/s
   * @param angular The angular speed of the robot in rad/s
   */
  void setSpeed(float linear, float angular);

  /**
   * Get the current odometry.
   */
  DiffDriveOdom getOdom();

  /**
   * Get the current wheel states
   */
  DiffDriveWheelStates getWheelStates();

  /**
   * Reset the odometry position.
   */
  void resetOdom();

  /**
   * Perform an update routine.
   * @param dt_ms Time elapsed since the last call to update function
   */
  void update(uint32_t dt_ms);

  void enable();
  void disable();

  // Wheel controllers
  WheelController wheel_FL;
  WheelController wheel_RL;
  WheelController wheel_FR;
  WheelController wheel_RR;

 private:
  DiffDriveOdom odom_;
  bool enabled_ = false;
  int last_command_time_remaining_;
  DiffDriveParams params_;
};
