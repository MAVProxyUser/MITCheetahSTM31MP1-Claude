#ifndef WALK_CONTROLLER_HPP
#define WALK_CONTROLLER_HPP

#include <RobotController.h>

/*!
 * Joint-space creep-gait walk for the Go1 (bypasses the model Jacobian/convention
 * issues that block the MPC's Cartesian control). Stands, then walks forward with a
 * statically-stable crawl (one leg swings at a time, three support). Optional
 * closed-loop waypoint: with a target set, it steers toward (tx,ty) and stops.
 */
class WalkController : public RobotController {
 public:
  WalkController() : RobotController() {}
  virtual ~WalkController() {}
  virtual void initializeController() {}
  virtual void runController();
  virtual void updateVisualization() {}
  virtual ControlParameters* getUserControlParameters() { return nullptr; }
};

#endif
