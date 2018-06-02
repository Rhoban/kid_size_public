#include "moves/AutonomousPlaying.h"

#include "moves/Head.h"
#include "moves/StandUp.h"
#include "moves/Walk.h"


#include <services/DecisionService.h>
#include <services/LocalisationService.h>

#include <rhoban_utils/logging/logger.h>

#define STATE_WAITING  "waiting"
#define STATE_PLAYING  "playing"
#define STATE_STANDUP  "standing_up"
#define STATE_HANDLED  "handled"

static rhoban_utils::Logger logger("AutonomousPlaying");


AutonomousPlaying::AutonomousPlaying(Walk *walk, StandUp *standup)
  : walk(walk), standup(standup)
{
  initializeBinding();
  // State
  bind->bindNew("state", STM::state, RhIO::Bind::PushOnly)
    ->comment("State of the AutonomousPlaying STM");

}

std::string AutonomousPlaying::getName()
{
  return "autonomous_playing";
}

void AutonomousPlaying::onStart()
{
  bind->pull();
  standup_try = 0;
  setState(STATE_WAITING);
  startMove("head", 0.5);
  walk->control(false);
}

void AutonomousPlaying::onStop()
{
  stopMove("head");
  setState(STATE_WAITING);
}

void AutonomousPlaying::step(float elapsed)
{
  auto &decision = getServices()->decision;
  bind->pull();

  // Detect if the robot is being handled
  if (decision->handled) {
    setState(STATE_HANDLED);
  } else if (state == STATE_HANDLED) {
    setState(STATE_WAITING);
  }

  t += elapsed;

  // Fall recovery start from waiting state
  if (state == STATE_WAITING && decision->isFallen) {
    logger.log("Robot has fallen, standing up");
    setState(STATE_STANDUP);
  }

  // If robot was standing up standup has finished, jump to Waiting state
  if (state == STATE_STANDUP && standup->over) {
    logger.log("Standup has finished, back to Waiting");
    setState(STATE_WAITING);
  }

  // If robot was playing and it fell, go to waiting (then it will go to standup)
  if (decision->isFallen && (state == STATE_PLAYING)) {
    logger.log("Robot has fallen, going to wait");
    setState(STATE_WAITING);
  }

  // Exiting buffer state (waiting)
  if (state == STATE_WAITING && !decision->handled && !decision->isFallen) {
    setState(STATE_PLAYING);
  }

  // Spam stop orders to walk when waiting or handled
  if (state == STATE_WAITING || state == STATE_HANDLED) {
    walk->control(false);
  }

  bind->push();
}


void AutonomousPlaying::enterState(std::string state)
{ 
    t = 0;

    Head * head = (Head*)getMoves()->getMove("head");
    // Not scanning only if the robot is handled
    if (state == STATE_HANDLED) {
        head->setDisabled(true);
    } else {
        head->setDisabled(false);
    }

    // Handling StandUp related stuff
    if (state == STATE_STANDUP) {
        standup->setLayDown(false);
        stopMove("walk", 0.3);
        startMove("standup", 0.0);
        standup->trying = standup_try;
    } else {
        startMove("walk", 1.0);
        walk->control(false);
    }

    if (state == STATE_PLAYING) {
      startMove("playing", 0.0);
    }
}

void AutonomousPlaying::exitState(std::string state)
{
    auto &decision = getServices()->decision;

    // After standing up:
    // - Apply a fallReset on filters
    // - Stop the standup move
    if (state == STATE_STANDUP) {
        auto loc = getServices()->localisation;
        loc->fallReset();
        stopMove("standup", 0.0);
        // Used to slow next standup if first one failed
        if (decision->isFallen) {
            standup_try++;
        } else {
            standup_try = 0;
        }
    }

    // Stop main move when leaving playing
    if (state == STATE_PLAYING) {
      stopMove("playing", 0.0);
    }
}

