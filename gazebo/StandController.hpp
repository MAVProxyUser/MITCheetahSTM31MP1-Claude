#ifndef STAND_CONTROLLER_HPP
#define STAND_CONTROLLER_HPP

#include <RobotController.h>

/*!
 * Minimal joint-space stand controller for SITL bring-up: holds a fixed stance
 * with per-joint PD (abad, hip, knee). Enough to see the Go1 stand in Gazebo and
 * to validate the joint sign/offset mapping. No user parameters.
 */
class StandController : public RobotController {
 public:
  StandController() : RobotController() {}
  virtual ~StandController() {}
  virtual void initializeController() {}
  virtual void runController();
  virtual void updateVisualization() {}
  virtual ControlParameters* getUserControlParameters() { return nullptr; }
};

#endif
