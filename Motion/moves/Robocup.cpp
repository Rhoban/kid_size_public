#include <math.h>
#include "Placer.h"
#include "Robocup.h"
#include <RhIO.hpp>
#include <services/RefereeService.h>
#include <services/LocalisationService.h>
#include <services/DecisionService.h>
#include <scheduler/MoveScheduler.h>
#include <services/CaptainService.h>
#include "StandUp.h"
#include "Head.h"
#include "Walk.h"
#include "Arms.h"

#include <rhoban_utils/logging/logger.h>

#define STATE_INITIAL "initial"
#define STATE_WAITING "waiting"
#define STATE_PLAYING "playing"
#define STATE_STANDUP "standing_up"
#define STATE_STOPPING "stopping"
#define STATE_PLACING "placing"
#define STATE_PENALIZED "penalized"
#define STATE_SERVING_PENALTY "serving_penalty"
#define STATE_GIVE_UP "give_up"
#define STATE_FINISHED "finished"

static rhoban_utils::Logger logger("RobocupSTM");

using namespace hl_communication;
using namespace rhoban_geometry;
using namespace rhoban_team_play;

Robocup::Robocup(Walk* walk, StandUp* standup, Placer* placer, Arms* arms)
  : walk(walk), standup(standup), placer(placer), arms(arms)
{
  initializeBinding();
  // State
  bind->bindNew("state", STM::state, RhIO::Bind::PushOnly)->comment("State of the Robocup STM");
  bind->bindNew("goalKeeper", goalKeeper, RhIO::Bind::PullOnly)->comment("Am I a goal keeper?")->defaultValue(false);
  bind->bindNew("freeKicker", freeKicker, RhIO::Bind::PullOnly)
      ->comment("Am I the free kicker performer?")
      ->defaultValue(false);
  // Initial locations for autoPlacing (Position at which the robot is dropped)
  bind->bindNew("autoStartX", autoStartX, RhIO::Bind::PullOnly)
      ->comment("Start of the robot during ready phase [m]")
      ->defaultValue(-1.5);
  bind->bindNew("autoStartY", autoStartY, RhIO::Bind::PullOnly)
      ->comment("Start of the robot during ready phase [m]")
      ->defaultValue(3);
  bind->bindNew("autoStartAzimuth", autoStartAzimuth, RhIO::Bind::PullOnly)
      ->comment("Initial orientation of the robot during ready phase [deg]")
      ->defaultValue(-90);
  // Target for autoPlacing
  bind->bindNew("autoTargetX", autoTargetX, RhIO::Bind::PullOnly)
      ->comment("Target of the robot during ready phase [m]")
      ->defaultValue(-1.5);
  bind->bindNew("autoTargetY", autoTargetY, RhIO::Bind::PullOnly)
      ->comment("Target of the robot during ready phase [m]")
      ->defaultValue(0);
  // Target location for the kicker
  bind->bindNew("freeKickX", freeKickX, RhIO::Bind::PullOnly)
      ->comment("Target of the robot performing freeKick during ready phase [m]")
      ->defaultValue(-0.5);
  bind->bindNew("freeKickY", freeKickY, RhIO::Bind::PullOnly)
      ->comment("Target of the robot performing freeKick during ready phase [m]")
      ->defaultValue(0);
  // Time since vision
  bind->bindNew("timeSinceVisionInactive", timeSinceVisionInactive, RhIO::Bind::PushOnly);
  bind->bindNew("timeSinceNoConsistency", timeSinceNoConsistency, RhIO::Bind::PushOnly);
}

std::string Robocup::getName()
{
  return "robocup";
}

void Robocup::onStart()
{
  bind->pull();

  standup_try = 0;
  setState(STATE_WAITING);
  startMove("head", 0.5);
  lastRefereePlaying = false;
  rememberStart = false;
  wasHandled = true;
  isHandled = true;
  walk->control(false);

  timeSinceNoConsistency = 0;
  timeSinceVisionInactive = 0;
}

void Robocup::onStop()
{
  stopMove("head");
  setState(STATE_STOPPING);
}

void Robocup::applyGameState()
{
  auto referee = getServices()->referee;

  // If we are at the beginning of the game, jump to state_initial
  if (state != STATE_INITIAL && referee->isInitialPhase())
  {
    setState(STATE_INITIAL);
  }

  if (state != STATE_FINISHED && referee->isFinishedPhase())
  {
    setState(STATE_FINISHED);
  }

  if (state != STATE_PLACING && referee->isPlacingPhase())
  {
    setState(STATE_PLACING);
  }

  // If mode is freeze, jump to waiting state
  if (state != STATE_WAITING && referee->isFreezePhase())
  {
    setState(STATE_WAITING);
  }

  // If we are allowed to play while we were not allowed previously
  if (!referee->isFreezePhase() && referee->isPlaying() && state == STATE_WAITING)
  {
    setState(STATE_PLAYING);
  }
}

void Robocup::step(float elapsed)
{
  auto& decision = getServices()->decision;
  bind->pull();

  // Detect if the robot is being handled
  if (decision->handled)
  {
    wasHandled = true;
  }

  t += elapsed;
  auto referee = getServices()->referee;
  bool isPlaying = referee->isPlaying();
  bool isPenalized = referee->isPenalized();
  // If robot is handled, it can't be serving penalty
  bool isServingPenalty = referee->isServingPenalty() && !decision->handled;

  // We gave up, only going to penalized or initial
  if (state == STATE_GIVE_UP)
  {
    if (!wasHandled || (!referee->isPenalized() && !referee->isInitialPhase()))
    {
      // Waiting to go back to either penalized or initial phase
      return;
    }
    else
    {
      stopMove("standup", 0.0);
    }
  }

  // We artificially go to penalized when handled in freeze phase so the border reset will
  // apply when dropped
  if (state == STATE_WAITING && referee->isFreezePhase() && decision->handled)
  {
    setState(STATE_PENALIZED);
  }

  /// Let standup finish if it started, otherwise go to penalized state if
  /// referee asks to
  if (state != STATE_STANDUP && state != STATE_PENALIZED && isPenalized && !isServingPenalty)
  {
    setState(STATE_PENALIZED);
  }
  // Transition from penalized to serving_penalty waits both, stopping handle + referee signal
  if (state == STATE_PENALIZED && isServingPenalty)
  {
    setState(STATE_SERVING_PENALTY);
  }
  /// Automatically going through WAITING when exiting penalized or serving_penalty
  if ((state == STATE_PENALIZED || state == STATE_SERVING_PENALTY) && !isPenalized && !decision->handled)
  {
    setState(STATE_WAITING);
  }

  // Fall recovery start from waiting state
  if (state == STATE_WAITING && decision->isFallen)
  {
    setState(STATE_STANDUP);
  }
  // Only apply gameState if robot is not standing up and not penalized
  if (state != STATE_STANDUP && state != STATE_PENALIZED && state != STATE_SERVING_PENALTY)
  {
    applyGameState();
  }
  // If robot was standing up, standup has finished, jump to Waiting state
  if (state == STATE_STANDUP && standup->over)
  {
    setState(STATE_WAITING);
  }
  // When the robot falls, go through waiting state (TODO, check if its necessary)
  if (decision->isFallen && (state == STATE_PLAYING || state == STATE_PLACING))
  {
    setState(STATE_WAITING);
  }
  // If the robot is being handled during the placing, go to waiting
  if (state == STATE_PLACING)
  {
    // Forwarding the captain order to the target
    auto captain = getServices()->captain;
    StrategyOrder order = captain->getMyOrder();
    const PoseDistribution& target = order.target_pose();
    placer->goTo(target.position().x(), target.position().y(), target.dir().mean());
  }

  // Spam stop orders to walk when waiting
  if (state == STATE_WAITING)
  {
    walk->control(false);
  }

  if (state == STATE_INITIAL || state == STATE_PLAYING)
  {
    wasHandled = false;
  }

  auto loc = getServices()->localisation;
  if (state == STATE_PLAYING)
  {
    // Vision is inactive
    if (!loc->isVisionActive())
    {
      timeSinceVisionInactive += elapsed;
      if (timeSinceVisionInactive > 30)
      {
        setState(STATE_GIVE_UP);
        logger.log("I lost my camera for more than 30s, giving up!");
      }
    }
    else
    {
      timeSinceVisionInactive = 0;
    }

    if (loc->fieldConsistency <= 0.1 && loc->consistencyEnabled)
    {
      timeSinceNoConsistency += elapsed;

      if (timeSinceNoConsistency > 15)
      {
        setState(STATE_GIVE_UP);
        logger.log("I lost my localization for more than 15s, giving up!");
      }
    }
    else
    {
      timeSinceNoConsistency = 0;
    }
    /*
    // We are attacking ourself, stop this!
    if (decision->isSelfAttacking) {
        setState(STATE_GIVE_UP);
        logger.log("I am attacking my own team?! Giving up");
    }
    */
  }
  else
  {
    timeSinceNoConsistency = 0;
    timeSinceVisionInactive = 0;
  }

  // Backuping old values
  lastRefereePlaying = isPlaying && !isPenalized;

  bool handled = decision->handled;
  if (!handled && isHandled && state == STATE_INITIAL)
  {
    // Note: we mostly do a reset here, as well as when exiting initial state, to be sure that  the robot is
    // properly positionned in the monitoring
    logger.log("Putting me on the floor in initial state, applying initial custom field reset");
    double locationNoise = 0.3;
    double azimuthNoise = 10;
    loc->customFieldReset(autoStartX, autoStartY, locationNoise, autoStartAzimuth, azimuthNoise);
  }
  isHandled = handled;

  bind->push();
}

void Robocup::enterState(std::string state)
{
  logger.log("Entering state %s", state.c_str());
  t = 0;

  Head* head = (Head*)getMoves()->getMove("head");
  // Not scanning only if the robot is penalized or game finished
  if (state == STATE_PENALIZED || state == STATE_FINISHED)
  {
    head->setDisabled(true);
  }
  else
  {
    head->setDisabled(false);
  }

  auto& decision = getServices()->decision;

  if (!decision->isThrowInRunning)
  {
    // Starts or stops the safe arms roll used for accessing the hotswap
    if (state == STATE_INITIAL || state == STATE_PENALIZED || state == STATE_FINISHED)
    {
      arms->setArms(Arms::ArmsState::ArmsMaintenance);
    }
    else
    {
      arms->setArms(Arms::ArmsState::ArmsEnabled);
    }
  }

  if (state == STATE_STANDUP)
  {
    standup->setLayDown(false);
    startMove("standup", 0.0);
    standup->trying = standup_try;
  }
  else
  {
    walk->control(false);
  }

  if (state == STATE_GIVE_UP)
  {
    standup->setLayDown(true);
    walk->control(false);
    startMove("standup", 0.0);
  }

  if (state == STATE_PLACING)
  {
    walk->control(true);
    startMove("placer");
    logger.log("Starting placer");
  }

  if (state == STATE_PLAYING)
  {
    rememberStart = false;
    if (goalKeeper)
    {
      startMove("goal_keeper", 0.0);
    }
    else
    {
      startMove("playing", 0.0);
    }
  }
}

void Robocup::exitState(std::string state)
{
  logger.log("Exiting state %s", state.c_str());

  auto& decision = getServices()->decision;
  LocalisationService* loc = getServices()->localisation;

  // After standing up:
  // - Apply a fallReset on filters
  // - Stop the standup move
  if (state == STATE_STANDUP)
  {
    auto loc = getServices()->localisation;
    loc->fallReset();
    stopMove("standup", 0.0);
    // Used to slow next standup if first one failed
    if (decision->isFallen)
    {
      standup_try++;
    }
    else
    {
      standup_try = 0;
    }
  }

  if (state == STATE_PENALIZED)
  {
    if (wasHandled)
    {
      // Robot was penalized and handled, resetting the filter to the borders of the field
      logger.log("Playing from bordersReset (penalized)");
      loc->bordersReset();
    }
    else
    {
      // Maybe the referee misclicked us, if we was not handled we should not reset the filters to the border
      logger.log("I am unpenalized but I was not handled, not resetting filters");
    }
  }

  // Stop main move when leaving playing
  if (state == STATE_PLAYING)
  {
    if (goalKeeper)
    {
      stopMove("goal_keeper", 0.0);
    }
    else
    {
      stopMove("playing", 0.0);
    }
  }

  // Destroy the placer
  if (state == STATE_PLACING)
  {
    stopMove("placer", 0.0);
    walk->control(false);
  }

  // End the player move
  if (state == STATE_PLAYING)
  {
    if (goalKeeper)
    {
      stopMove("goal_keeper", 0.0);
    }
    else
    {
      stopMove("playing", 0.0);
    }
  }

  if (state == STATE_INITIAL)
  {
    logger.log("Exiting initial state, applying initial custom reset");
    double locationNoise = 0.3;
    double azimuthNoise = 10;
    loc->customFieldReset(autoStartX, autoStartY, locationNoise, autoStartAzimuth, azimuthNoise);
  }
}

bool Robocup::isGoalKeeper() const
{
  return goalKeeper;
}
