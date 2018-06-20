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
  rhoban_utils::Control stepper, lateraler, aligner;
    
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
  
  // Potential repulsion
  double repulsion;

  // °/m
  double degsPerMeter;

  // Don't walk
  bool dontWalk;

  // Acceptation threshold
  double distanceThreshold, angleThreshold;

  // Gains
  double stepP, lateralI, stepI, stepPunch, lateralP;

  // Current tolerance used
  double currentTolerance;

  void enterState(std::string state);
  void exitState(std::string state);

  Eigen::Vector3d computeLastStepDelta(
      const std::string& kickName, 
      bool isRightFoot);
};
