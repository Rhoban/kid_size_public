#include "ReactiveKicker.h"

#include "Walk.h"
#include "Kick.h"
#include "Head.h"
#include <services/LocalisationService.h>

using namespace rhoban_geometry;
using namespace rhoban_utils;

ReactiveKicker::ReactiveKicker(Walk* walk, Kick* kick, Head* head) : ApproachMove(walk, kick), head(head)
{
  Move::initializeBinding();
  ApproachMove::initBindings();

  bind->bindNew("anticipationMean", anticipationMean, RhIO::Bind::PullOnly)->defaultValue(0.35);
  bind->bindNew("anticipationDelta", anticipationDelta, RhIO::Bind::PullOnly)->defaultValue(0.04);

  bind->bindNew("is_kicking", is_kicking, RhIO::Bind::PushOnly)
      ->defaultValue(false)
      ->comment("Is the robot currently kicking?");

  bind->bindNew("kick_pause_time", kickPauseTime, RhIO::Bind::PullOnly)->defaultValue(1.8);

  bind->bindNew("useRightFoot", useRightFoot, RhIO::Bind::PullOnly)->defaultValue(true);

  bind->pull();
}

std::string ReactiveKicker::getName()
{
  return "reactive_kicker";
}

void ReactiveKicker::onStart()
{
  bind->pull();

  // Simply use only classic currently
  is_kicking = false;
  kick_score = 0;
  expectedKick = "classic";
  kickRight = useRightFoot;

  head->setDisabled(false);
  head->setForceTrack(true);

  kick->set(!useRightFoot, "classic", true, kickPauseTime);
  startMove("kick", 0.0);
}

void ReactiveKicker::onStop()
{
  head->setForceTrack(false);
}

rhoban_utils::Angle ReactiveKicker::getKickCap()
{
  return 0;
}

void ReactiveKicker::step(float elapsed)
{
  // Pull variables from RhIO
  bind->pull();

  // Handle the case where we are currently kicking
  if (is_kicking)
  {
    if (!kick->isRunning())
    {
      this->Move::stop();
    }
    return;
  }

  for (double anticipation = anticipationMean - anticipationDelta; anticipation < anticipationMean + anticipationDelta;
       anticipation += 0.005)
  {
    // When is the robot expected to perform the kick
    TimeStamp kick_time = TimeStamp::now().addMS(anticipation * 1000);

    // First: retrieving ball position in the future
    LocalisationService* loc = getServices()->localisation;
    Point future_ball_loc = loc->getPredictedBallSelf(kick_time, true);

    // Updating the kick score using the future ball position instead of present one
    kick_gain = 1000;
    // std::cout << "Future ball: " << future_ball_loc << std::endl;
    updateKickScore(elapsed, future_ball_loc);

    // Only use classic
    if (kick_score >= 1.0)
    {
      kick->unpause();
      is_kicking = true;
      break;
    }
  }

  bind->push();
}
