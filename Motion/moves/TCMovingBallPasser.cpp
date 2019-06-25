#include "TCMovingBallPasser.h"

#include <services/LocalisationService.h>
#include <services/DecisionService.h>
#include <robocup_referee/constants.h>

#include "Head.h"
#include "Walk.h"

using namespace rhoban_geometry;
using namespace rhoban_utils;

TCMovingBallPasser::TCMovingBallPasser(Walk* walk, Head* head) : walk(walk), head(head)
{
  Move::initializeBinding();

  bind->bindNew("yPositive", yPositive, RhIO::Bind::PullOnly)->defaultValue(true);
  bind->bindNew("kickDirection", kickDirection, RhIO::Bind::PullOnly)->defaultValue(-118);

  bind->pull();
}

std::string TCMovingBallPasser::getName()
{
  return "tc_moving_ball_passer";
}

void TCMovingBallPasser::onStart()
{
  isRunning = false;
  bind->pull();
  head->setDisabled(false);

  // XXX: This is hacky
  RhIO::Root.setFloat("/moves/walk/maxStep", 0.06);
  RhIO::Root.setFloat("/moves/walk/maxLateral", 0.03);
  RhIO::Root.setFloat("/moves/walk/maxRotation", 15);
}

void TCMovingBallPasser::onStop()
{
  stopMove("approach_potential", 0);
}

void TCMovingBallPasser::step(float elapsed)
{
  // Pull variables from RhIO
  bind->pull();
  LocalisationService* localisation = getServices()->localisation;
  DecisionService* decision = getServices()->decision;

  // Asking for a classic kick with tuned direction
  allowed_kicks.clear();
  allowed_kicks.push_back("classic");
  tolerance = 0;
  kick_dir = kickDirection;

  t += elapsed;

  if (isRunning)
  {
    if (decision->handled)
    {
      isRunning = false;
      stopMove("approach_potential", 0);
    }
    else
    {
      if (t > 3)
      {
        if (decision->isBallQualityGood)
        {
          startMove("approach_potential", 0);
        }
        else
        {
          stopMove("approach_potential", 0);
        }
      }
    }
  }
  else
  {
    if (!decision->handled)
    {
      isRunning = true;
      localisation->customFieldReset(
          robocup_referee::Constants::field.field_length / 2.0 - robocup_referee::Constants::field.goal_area_length,
          (yPositive ? 1 : -1) * robocup_referee::Constants::field.field_width / 2.0, 0.2, 0, 5);
      t = 0;
    }
  }

  bind->push();
}
