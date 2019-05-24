#pragma once

#include <thread>
#include <mutex>
#include "moves/KickController.h"
#include <rhoban_utils/control/control.h>
#include <services/TeamPlayService.h>

#include "rhoban_csa_mdp/core/policy.h"
#include "rhoban_geometry/point.h"
#include "rhoban_utils/angle.h"
#include <strategy/KickStrategy.hpp>
#include <strategy/KickValueIteration.hpp>

class Walk;

class MCKickController : public KickController
{
public:
  MCKickController();
  std::string getName();

  void onStart();
  void onStop();
  void execute();

  void step(float elapsed);

  KickValueIteration kickValueIteration;
  
  // Avoid the opponents ?
  bool avoidOpponents;
  
protected:
  KickStrategy strategy;
  std::string strategyFile;
  KickStrategy::Action action;
  std::thread* thread;

  bool forceUpdate;
  rhoban_geometry::Point lastUpdateBall;


  // The collection of available kicks
  csa_mdp::KickModelCollection kmc;

  // Updating the target action
  void updateAction();

  std::string cmdReloadStrategy();

  bool shouldReload;

  std::mutex mutex;

  // Data are copied between thread and tick to avoid ressource access issues
  std::vector<rhoban_geometry::Point> _opponentsField;
  std::map<int, Eigen::Vector3d> _teamMatesField;
  rhoban_geometry::Point _ballField;
  rhoban_geometry::Point _robotField;
  double _opponentsRadius;

  KickStrategy::Action _bestAction;
};
