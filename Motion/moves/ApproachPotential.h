#pragma once

#include "moves/ApproachMove.h"
#include <rhoban_utils/control/control.h>
#include <services/TeamPlayService.h>

class Walk;
class KickController;
class ApproachPotential : public ApproachMove
{
public:
  struct Target
  {
    Target();
    Target(rhoban_geometry::Point position, rhoban_utils::Angle yaw, bool rightKick, std::string kickName,
           double tolerance);
    rhoban_geometry::Point position;
    rhoban_utils::Angle yaw;
    bool rightKick;
    std::string kickName;
    double tolerance;
  };

  virtual rhoban_utils::Angle getKickCap();

  ApproachPotential(Walk* walk);
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
  void getControl(const Target& target, const rhoban_geometry::Point& ball, double& x, double& y, double& yaw);

  // Foot choice
  bool left;
  double lastFootChoice;

  double ballX, ballY;

  // Potential repulsion
  double repulsion;

  // Don't walk
  bool dontWalk;

  // Gains
  double stepP, lateralI, stepI, stepPunch, lateralP;

  // Current tolerance used
  double currentTolerance;

  double placementDistance;

  void enterState(std::string state);
  void exitState(std::string state);
};
