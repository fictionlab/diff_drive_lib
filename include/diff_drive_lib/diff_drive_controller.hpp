#pragma once

#include <vector>

#include "diff_drive_lib/robot_controller.hpp"

namespace diff_drive_lib {

class DiffDriveController : public RobotController{
 public:
  using RobotController::RobotController;
  // DiffDriveController(const DiffDriveConfiguration& dd_conf);
  void setSpeed(float linear_x, float linear_y, float angular) override;

  void update(uint32_t dt_ms) override;
};

}  // namespace diff_drive_lib