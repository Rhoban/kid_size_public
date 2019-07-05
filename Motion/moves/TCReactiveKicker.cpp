#include "TCReactiveKicker.h"

#include <services/LocalisationService.h>
#include <services/DecisionService.h>
#include <robocup_referee/constants.h>

#include "Head.h"
#include "Walk.h"

using namespace rhoban_geometry;
using namespace rhoban_utils;

TCReactiveKicker::TCReactiveKicker(Walk* walk, Head* head) : walk(walk), head(head)
{
  Move::initializeBinding();

  bind->pull();
}

std::string TCReactiveKicker::getName()
{
  return "tc_reactive_kicker";
}

void TCReactiveKicker::onStart()
{
  isRunning = false;
  bind->pull();
  head->setDisabled(false);

  RhIO::Root.setInt("/Vision/ballByII/maxRois", 32);
  RhIO::Root.setFloat("/moves/walk/maxStep", 0.06);
}

void TCReactiveKicker::onStop()
{
}

void TCReactiveKicker::step(float elapsed)
{
  // Pull variables from RhIO
  bind->pull();
  // LocalisationService* localisation = getServices()->localisation;
  DecisionService* decision = getServices()->decision;

  t += elapsed;

  if (isRunning)
  {
    if (decision->handled)
    {
      isRunning = false;
      stopMove("reactive_kicker", 0);
      stopMove("kick", 3);
    }
    else
    {
      if (t > 3)
      {
        startMove("reactive_kicker", 0);
      }
    }
  }
  else
  {
    if (!decision->handled)
    {
      isRunning = true;
      t = 0;
    }
  }

  bind->push();
}
