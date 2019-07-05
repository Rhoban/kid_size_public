#include "TCHighKick.h"

#include <services/LocalisationService.h>
#include <services/DecisionService.h>
#include <robocup_referee/constants.h>

#include "Head.h"
#include "Walk.h"

using namespace rhoban_geometry;
using namespace rhoban_utils;

TCHighKick::TCHighKick(Walk* walk, Head* head) : walk(walk), head(head)
{
  Move::initializeBinding();

  strategy.fromJson("kickStrategy_with_grass.json");

  bind->pull();
}

std::string TCHighKick::getName()
{
  return "tc_high_kick";
}

void TCHighKick::onStart()
{
  isRunning = false;
  bind->pull();
  head->setDisabled(false);
  RhIO::Root.setFloat("/moves/walk/maxStep", 0.06);
}

void TCHighKick::onStop()
{
  stopMove("approach_potential", 0);
}

void TCHighKick::step(float elapsed)
{
  // Pull variables from RhIO
  bind->pull();
  LocalisationService* localisation = getServices()->localisation;
  DecisionService* decision = getServices()->decision;

  // Getting best action from strategy to get target orientation
  auto ball = localisation->getBallPosField();
  auto bestAction = strategy.actionFor(ball.x, ball.y);
  kick_dir = rad2deg(bestAction.orientation);
  tolerance = 0;

  allowed_kicks.clear();
  if (ball.x <
      robocup_referee::Constants::field.field_length / 2.0 - robocup_referee::Constants::field.goal_area_length)
  {
    allowed_kicks.push_back("small");
  }
  else
  {
    allowed_kicks.push_back("high_kick");
  }

  t += elapsed;
  if (isRunning)
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

    if (decision->handled)
    {
      head->setDisabled(true);
      stopMove("approach_potential", 0);
      isRunning = false;
    }
  }
  else
  {
    if (!decision->handled)
    {
      t = 0;
      isRunning = true;
      head->setDisabled(false);
      head->setForceTrack(true);
      localisation->customFieldReset(robocup_referee::Constants::field.field_length / 2.0 -
                                         robocup_referee::Constants::field.penalty_mark_dist - 0.5,
                                     0, 0.2, 0, 5);
    }
  }

  bind->push();
}
