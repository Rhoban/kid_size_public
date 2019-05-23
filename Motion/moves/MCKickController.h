#pragma once

#include "moves/KickController.h"
#include <rhoban_utils/control/control.h>
#include <services/TeamPlayService.h>

#include "rhoban_csa_mdp/core/policy.h"
#include "rhoban_geometry/point.h"
#include "rhoban_utils/angle.h"
#include <strategy/KickStrategy.hpp>

class Walk;

class MCKickController : public KickController
{
public:
  MCKickController();
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

  // Avoid the opponents ?
  bool avoidOpponents;

  // The collection of available kicks
  csa_mdp::KickModelCollection kmc;

  // Updating the target action
  void updateAction();

  std::string cmdReloadStrategy();
};
