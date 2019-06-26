#include "TCPushRecovery.h"

#include <services/LocalisationService.h>
#include <services/DecisionService.h>
#include <robocup_referee/constants.h>

#include "Head.h"
#include "Walk.h"
#include "Kick.h"

using namespace rhoban_geometry;
using namespace rhoban_utils;

TCPushRecovery::TCPushRecovery(Walk* walk, Head* head, Kick* kick) : walk(walk), head(head), kick(kick)
{
  Move::initializeBinding();

  bind->pull();
}

std::string TCPushRecovery::getName()
{
  return "tc_push_recovery";
}

void TCPushRecovery::onStart()
{
  isRunning = false;
  bind->pull();
  head->setDisabled(true);
  isRunning = false;

  RhIO::Root.setFloat("/moves/walk/frequency", 1.7);
  RhIO::Root.setFloat("/moves/walk/trunkZOffset", 0.035);
}

void TCPushRecovery::onStop()
{
  walk->control(false);
}

void TCPushRecovery::step(float elapsed)
{
  // Pull variables from RhIO
  bind->pull();

  DecisionService* decision = getServices()->decision;

  if (!decision->handled)
  {
    t += elapsed;
    if (!isRunning && t > 1)
    {
      walk->control(true);
    }
  }
  else
  {
    walk->control(false);
    t = 0;
    isRunning = false;
  }

  bind->push();
}
