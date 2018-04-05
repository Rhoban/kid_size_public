#include "ApproachMove.h"

#include "rhoban_utils/logging/logger.h"

#include "KickController.h"
#include "Walk.h"
#include "scheduler/MoveScheduler.h"

#include <services/LocalisationService.h>
#include <services/TeamPlayService.h>
#include <services/StrategyService.h>

#include <iostream>

using csa_mdp::KickZone;
using namespace rhoban_utils;
using namespace rhoban_geometry;
using namespace rhoban_team_play;

static rhoban_utils::Logger logger("ApproachMove");

ApproachMove::ApproachMove(Walk * walk_)
  : walk(walk_), expectedKick("classic")
{
  kmc.loadFile();
}

void ApproachMove::initBindings()
{
  bind->bindNew("expectedKick", expectedKick, RhIO::Bind::PushOnly)
    ->defaultValue(expectedKick);
  bind->bindNew("kickRight", kickRight, RhIO::Bind::PushOnly)->defaultValue(0.0)
    ->comment("Does next kick use right foot?");
  bind->bindNew("kickScore", kick_score, RhIO::Bind::PushOnly)->defaultValue(0.0)
    ->comment("When score reaches 1, robot is allowed to shoot");
  bind->bindNew("kickGain", kick_gain, RhIO::Bind::PullOnly)->defaultValue(2.0)
    ->comment("IncreaseRate when ball is kickable (score increases by elapsed * kick_gain)")
    ->persisted(true);
  bind->bindNew("noKickGain", no_kick_gain, RhIO::Bind::PullOnly)->defaultValue(0.5)
    ->comment("DecreaseRate when ball is not kickable (score decreases by elapsed * no_kick_gain)")
    ->persisted(true);
  bind->bindNew("enableLateral", enableLateral, RhIO::Bind::PullOnly)
    ->defaultValue(true)->comment("Enable the lateral kicks"
                                  "(this option is only used if there is no kick controller)")
    ->persisted(true);

}

void ApproachMove::onStart()
{
  getServices()->teamPlay->selfInfo().state = Playing;
}

void ApproachMove::onStop()
{
  getServices()->teamPlay->selfInfo().state = Inactive;
}

Angle ApproachMove::getKickCap()
{
  LocalisationService * loc = getServices()->localisation;
  Angle playerDir(rad2deg(loc->getFieldOrientation()));

  if (useKickController()) {
    Angle kickDirField = getKickController()->getKickDir();
    return  kickDirField - playerDir;
  } else {
    return (loc->getGoalPosField() - loc->getBallPosField()).getTheta() - playerDir;
  }
}

double ApproachMove::getKickTolerance()
{
    if (useKickController()) {
        return getKickController()->getTolerance();
    } else {
        return 0;
    }
}

const KickController * ApproachMove::getKickController() const
{
  StrategyService *strategy = getServices()->strategy;
  return strategy->getActiveKickController();
}

std::vector<std::string> ApproachMove::getAllowedKicks()
{
  if (useKickController()) {
    return getKickController()->getAllowedKicks();
  }
  if (enableLateral) {
    return {"classic", "lateral"};
  }
  return {"classic"};
}

bool ApproachMove::isKickAllowed(const std::string & name)
{
  for (const std::string & kickName : getAllowedKicks()) {
    if (name == kickName) return true;
  }
  return false;
}

const std::string & ApproachMove::getExpectedKick() const
{
  return expectedKick;
}

void ApproachMove::requestKick()
{
  // Building message:
  std::ostringstream oss;
  oss << "Kick in '" << getName() << "': " << expectedKick
      << " with " << (kickRight ? "right" : "left") << " foot";
  logger.log("%s", oss.str().c_str());
  // Sending kick order to walk
  walk->kick(kickRight, expectedKick);
}

bool ApproachMove::useKickController() const
{
  return getKickController() != NULL && getKickController()->isRunning();
}


void ApproachMove::updateKickScore(double elapsed)
{
  LocalisationService * loc = getScheduler()->getServices()->localisation;
  Point ball_pos = loc->getBallPosSelf();
  updateKickScore(elapsed, ball_pos);
}

void ApproachMove::updateKickScore(double elapsed, const Point & ball_pos)
{
  // Updating kick score
  const KickZone & kick_zone = kmc.getKickModel(expectedKick).getKickZone();
  double kick_dir_rad = deg2rad(getKickCap().getSignedValue());
  Eigen::Vector3d state(ball_pos.x, ball_pos.y, kick_dir_rad);
  // Debug
  std::ostringstream oss;
  oss << "Ball state: (" << state.transpose() << ") -> ";
  Eigen::Vector3d wished_pos = kick_zone.getWishedPos(kickRight); 
  oss << " (wished_pos: " << wished_pos.transpose() << ")";
  if (kick_zone.canKick(kickRight, state)) {
    kick_score += elapsed * kick_gain;
    logger.log("%s", oss.str().c_str());
  }
  else {
    kick_score -= elapsed * no_kick_gain;
    if (kick_zone.canKick(!kickRight, state)) {
      oss << " -> but can kick with other foot";
    }
  }
  // Bounding kick_score values
  kick_score = std::max(0.0, std::min(1.0, kick_score));
}
