#include "ReactiveKicker.h"

#include "Walk.h"
#include "Kick.h"
#include <services/LocalisationService.h>

using namespace rhoban_geometry;
using namespace rhoban_utils;

ReactiveKicker::ReactiveKicker(Walk* walk, Kick* kick) : ApproachMove(walk, kick)
{
  Move::initializeBinding();
  ApproachMove::initBindings();

  bind->bindNew("anticipation", anticipation, RhIO::Bind::PullOnly)
      ->defaultValue(1)
      ->minimum(0)
      ->maximum(2)
      ->comment("How much time is necessary between kick decision and ball contact [s]")
      ->persisted(true);

  bind->bindNew("is_kicking", is_kicking, RhIO::Bind::PushOnly)
      ->defaultValue(false)
      ->comment("Is the robot currently kicking?");

  bind->bindNew("kick_pause_time", kickPauseTime, RhIO::Bind::PullOnly)->defaultValue(1.8);

  bind->pull();
}

std::string ReactiveKicker::getName()
{
  return "reactive_kicker";
}

void ReactiveKicker::onStart()
{
  // Simply use only classic currently
  is_kicking = false;
  kick_score = 0;
  expectedKick = "classic";
  kickRight = true;

  kick->set(false, "classic", true, kickPauseTime);
  startMove("kick", 0.0);
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

  // When is the robot expected to perform the kick
  TimeStamp kick_time = TimeStamp::now().addMS(anticipation);

  // First: retrieving ball position in the future
  LocalisationService* loc = getServices()->localisation;
  Point future_ball_loc = loc->getPredictedBallSelf(kick_time);

  // Updating the kick score using the future ball position instead of present one
  kick_gain = 1000;
  std::cout << "Future ball: " << future_ball_loc << std::endl;
  updateKickScore(elapsed, future_ball_loc);

  // Only use classic
  if (kick_score >= 1.0)
  {
    kick->unpause();
    is_kicking = true;
  }

  bind->push();
}
