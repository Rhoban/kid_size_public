#include <functional>
#include <algorithm>
#include <rhoban_geometry/segment.h>
#include <rhoban_geometry/circle.h>
#include <services/LocalisationService.h>
#include <robocup_referee/constants.h>
#include "rhoban_utils/logging/logger.h"
#include "MCKickController.h"

using namespace rhoban_geometry;
using namespace rhoban_utils;
using namespace robocup_referee;

static rhoban_utils::Logger logger("MCKickController");

MCKickController::MCKickController()
{
  Move::initializeBinding();

  // Strategy file to load
  bind->bindNew("strategyFile", strategyFile)
      ->defaultValue("kickStrategy_with_grass.json")
      ->comment("Strategy file")
      ->persisted(true);

  // Should we avoid opponents
  bind->bindNew("avoidOpponents", avoidOpponents, RhIO::Bind::PullOnly)
      ->defaultValue(false)
      ->comment("Avoid the opponents?");

  // Reload strategy file
  bind->bindFunc("reloadStrategy", "Reload the strategy file", &MCKickController::cmdReloadStrategy, *this);

  bind->pull();

  // Loading strategy
  if (!strategy.fromJson(strategyFile))
  {
    logger.error("Can't load kick strategy file %s", strategyFile.c_str());
  }

  // Load available kicks
  kmc.loadFile();
}

std::string MCKickController::cmdReloadStrategy()
{
  bind->pull();
  std::stringstream ss;

  if (!strategy.fromJson(strategyFile))
  {
    ss << "Can't load strategy file " << strategyFile;
  }
  else
  {
    ss << "Strategy " << strategyFile << " loaded.";
  }

  return ss.str();
}

std::string MCKickController::getName()
{
  return "mc_kick_controler";
}

void MCKickController::onStart()
{
  bind->pull();
  forceUpdate = true;
}

void MCKickController::onStop()
{
}

void MCKickController::updateAction()
{
  // auto loc = getServices()->localisation;
  // Point ball = loc->getBallPosField();
  // action = strategy.actionFor(ball.x, ball.y);
  // tolerance = rad2deg(action.tolerance) / 2;

  // if (action.kick == "")
  // {
  //   return;
  // }

  // allowed_kicks.clear();
  // if (action.kick == "opportunist")
  // {
  //   allowed_kicks.push_back("classic");
  //   allowed_kicks.push_back("lateral");
  // }
  // else
  // {
  //   allowed_kicks.push_back(action.kick);
  // }

  // kick_dir = rad2deg(action.orientation);
}

void MCKickController::step(float elapsed)
{
  bind->pull();
}
