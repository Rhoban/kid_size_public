#include "TCJump.h"

#include <services/LocalisationService.h>
#include <services/DecisionService.h>
#include <robocup_referee/constants.h>

#include "Head.h"
#include "Walk.h"
#include "Kick.h"

using namespace rhoban_geometry;
using namespace rhoban_utils;

TCJump::TCJump(Walk* walk, Head* head, Kick* kick) : walk(walk), head(head), kick(kick)
{
  Move::initializeBinding();

  bind->pull();
}

std::string TCJump::getName()
{
  return "tc_jump";
}

void TCJump::onStart()
{
  isRunning = false;
  bind->pull();
  head->setDisabled(true);
  isRunning = false;
  t = 0;
}

void TCJump::onStop()
{
}

void TCJump::step(float elapsed)
{
  // Pull variables from RhIO
  bind->pull();

  DecisionService* decision = getServices()->decision;

  if (!decision->handled)
  {
    t += elapsed;
    if (!isRunning && t > 3)
    {
      isRunning = true;
      kick->set(true, "jump");
      startMove("kick", 0.0);
    }
  }
  else
  {
    t = 0;
    isRunning = false;
  }

  bind->push();
}
