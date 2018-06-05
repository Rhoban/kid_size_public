#include <services/TeamPlayService.h>

#include "StrategyService.h"
#include "moves/ApproachMove.h"
#include "moves/KickController.h"

#include "services/LocalisationService.h"

#include "scheduler/MoveScheduler.h"

#include "rhoban_utils/logging/logger.h"

using namespace rhoban_geometry;
using namespace rhoban_utils;

static rhoban_utils::Logger logger("StrategyService");

StrategyService::StrategyService()
  : bind("strategy"),
    active_approach("none"), active_kick_controler("none"),
    kick_target_x(0), kick_target_y(0), lastKick(TimeStamp::fromMS(0))
{
  kmc.loadFile();

  // Bind members
  // Default and active moves for approach and kick controler
  bind.bindNew("defaultApproach", default_approach, RhIO::Bind::PullOnly)
    ->defaultValue("approach_potential")->persisted(true);
  bind.bindNew("activeApproach", active_approach, RhIO::Bind::PushOnly)
    ->defaultValue("none");
  bind.bindNew("defaultKickController", default_kick_controler, RhIO::Bind::PullOnly)
    ->defaultValue("q_kick_controler")->persisted(true);
  bind.bindNew("grassOffset", grass_offset, RhIO::Bind::PullOnly)
    ->defaultValue(180)->persisted(true);
  bind.bindNew("activeKickController", active_kick_controler, RhIO::Bind::PushOnly)
    ->defaultValue("none");
  // Kick target
  bind.bindNew("kickTargetX", kick_target_x, RhIO::Bind::PushOnly)
    ->defaultValue(0);
  bind.bindNew("kickTargetY", kick_target_y, RhIO::Bind::PushOnly)
    ->defaultValue(0);
  bind.bindNew("kickControllerDir", kick_controler_dir, RhIO::Bind::PushOnly)
    ->defaultValue(0);
  // Kick tolerance
  bind.bindNew("kickTolerance", kick_tolerance, RhIO::Bind::PushOnly)
    ->defaultValue(0);
}

const std::string & StrategyService::getDefaultApproach() {
  return default_approach;
}

const std::string & StrategyService::getDefaultKickController() {
  return default_kick_controler;
}

const std::string & StrategyService::getActiveApproachName() {
  return active_approach;
}

const std::string & StrategyService::getActiveKickControllerName() {
  return default_kick_controler;
}

ApproachMove * StrategyService::getActiveApproach() {
  if (active_approach == "none" || active_approach == "conflict") {
    return nullptr;
  }
  return dynamic_cast<ApproachMove *>(getScheduler()->getMove(active_approach));
}

const KickController * StrategyService::getActiveKickController() const {
  if (active_kick_controler == "none" || active_kick_controler == "conflict") {
    return nullptr;
  }
  return dynamic_cast<const KickController *>(getScheduler()->getMove(active_kick_controler));
}

//Return none if no active move, conflict if several active moves and
template <class T>
std::string getActiveMove(const std::vector<std::string> & candidates,
                          MoveScheduler * scheduler)
{
  std::vector<std::string> active_moves;
  for (const std::string & name : candidates) {
    if (scheduler->getMove(name)->isRunning()) {
      active_moves.push_back(name);
    }
  }
  if (active_moves.size() == 0) return "none";
  if (active_moves.size() > 1) return "conflict";
  return active_moves[0];
}

bool StrategyService::tick(double elapsed)
{
  // auto teamPlay = getServices()->teamPlay;
  // auto &info = teamPlay->selfInfo();
  // XXX: Todo share the ball target X

  static std::vector<std::string> approach_candidates =
    {"approach", "approach_potential", "learned_approach"};
  static std::vector<std::string> kick_controler_candidates =
    {"kick_controler", "q_kick_controler", "clearing_kick_controler",
    "penalty_kick_controler"};


  bind.pull();

  active_approach = getActiveMove<ApproachMove *>(approach_candidates,
                                                  getScheduler());
  active_kick_controler = getActiveMove<KickController *>(kick_controler_candidates,
                                                         getScheduler());

  ApproachMove * approach = getActiveApproach();
  const KickController * kick_controler = getActiveKickController();
  bool kick_ok = false;

  // Updating the grass cone offset
  kmc.setGrassConeOffset(grass_offset);

  // Updating targets for kick
  if (approach != nullptr || kick_controler != nullptr) {
    kick_ok = true;
    LocalisationService * loc = getScheduler()->getServices()->localisation;
    std::string kick_name;
    Angle kick_dir_in_field;
    if (kick_controler != nullptr) {
        kick_tolerance = kick_controler->getTolerance();
        kick_controler_dir = kick_controler->getKickDir().getSignedValue();
    }

    if (approach != nullptr) {
        // Getting kick properties
        kick_name = approach->getExpectedKick();
        Angle kick_dir_in_robot = approach->getKickCap();
        Angle robot_in_field(rad2deg(loc->getFieldOrientation()));
        kick_dir_in_field = kick_dir_in_robot + robot_in_field;
    } else {
        auto kicks = kick_controler->getAllowedKicks();
        if (kicks.size()) {
            kick_name = kicks.front();
        } else {
            kick_ok = false;
        }
        kick_dir_in_field = kick_controler->getKickDir();
    }
    if (kick_ok) {
      const csa_mdp::KickModel & kick_model = kmc.getKickModel(kick_name);
      double kick_dir_rad = deg2rad(kick_dir_in_field.getSignedValue());
      // Computing final ball position
      Point ball_pos_point = loc->getBallPosField();
      Eigen::Vector2d ball_pos_field(ball_pos_point.x, ball_pos_point.y);
      Eigen::Vector2d target = kick_model.applyKick(ball_pos_field, kick_dir_rad);
      // Updating variables
      kick_target_x = target(0);
      kick_target_y = target(1);
    }
  }

  if (!kick_ok) {
    kick_target_x = 0;
    kick_target_y = 0;
  }

  // Publishing ball target in team play
  auto teamPlay = getServices()->teamPlay;
  auto &info = teamPlay->selfInfo();
  info.ballTargetX = kick_target_x;
  info.ballTargetY = kick_target_y;

  // Publishing time elapsed since last kick
  info.timeSinceLastKick = diffSec(lastKick, TimeStamp::now());

  bind.push();

  return true;
}

void StrategyService::announceKick() {
  lastKick = TimeStamp::now();
}
