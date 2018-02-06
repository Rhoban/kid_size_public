#pragma once

#include "moves/KickController.h"
#include <rhoban_utils/control/control.h>
#include <services/TeamPlayService.h>

#include "rhoban_csa_mdp/core/policy.h"
#include "rhoban_utils/angle.h"
#include <strategy/KickStrategy.hpp>

class Walk;

class QKickController : public KickController
{
public:
  QKickController();
  std::string getName();

  void onStart();
  void onStop();

  void step(float elapsed);

protected:
  KickStrategy strategy;
  std::string strategyFile;
  KickStrategy::Action action;

  bool forceUpdate;
  rhoban_geometry::Point lastUpdateBall;

  // Update when ball is more far than this value [m]
  double farUpdateDist;

  // Update if the ball has moved more than this value [m]
  double moveUpdateDist;

  // Avoid the opponents ?
  bool avoidOpponents;
        
  // The collection of available kicks
  csa_mdp::KickModelCollection kmc;

  // Updating the target action
  void updateAction();

  // Should
  bool shouldUpdate();

  std::string cmdReloadStrategy();
};
