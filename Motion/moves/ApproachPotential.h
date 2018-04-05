#pragma once

#include "moves/ApproachMove.h"
#include <rhoban_utils/control/control.h>
#include <services/TeamPlayService.h>

class Walk;
class KickController;
class ApproachPotential : public ApproachMove
{
public:
  struct Target {
    Target();
    Target(rhoban_geometry::Point position, rhoban_utils::Angle yaw,
           bool rightKick, std::string kickName, double tolerance);
    rhoban_geometry::Point position;
    rhoban_utils::Angle yaw;
    bool rightKick;
    std::string kickName;
    double tolerance;
  };

  virtual rhoban_utils::Angle getKickCap();

  ApproachPotential(Walk *walk);
  std::string getName();

  void onStart();
  void onStop();
  void step(float elapsed);

protected:
  std::vector<Target> targets;

  /**
   * Getting the control to send to the walk to reach a given target, knowing
   * where the ball is
   */
  void getControl(const Target &target, const rhoban_geometry::Point &ball,
                  double &x, double &y, double &yaw);

  // Foot choice
  bool left;
  double lastFootChoice;

  // °/m
  double degsPerMeter;

  // Target X/Y for the ball in the robot frame
  double targetX, targetY;

  // Don't walk
  bool dontWalk;

  // Acceptation threshold
  double distanceThreshold, angleThreshold;

  // Gains
  double stepP, rotationP, stepPunch;

  // Current tolerance used
  double currentTolerance;

  void enterState(std::string state);
  void exitState(std::string state);
};
