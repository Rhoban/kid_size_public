#include "ReactiveKicker.h"

#include "Walk.h"
#include <services/LocalisationService.h>

using namespace rhoban_geometry;
using namespace rhoban_utils;

ReactiveKicker::ReactiveKicker(Walk * walk) : ApproachMove(walk)
{
  Move::initializeBinding();
  ApproachMove::initBindings();
  bind->bindNew("anticipation", anticipation, RhIO::Bind::PullOnly)
    ->defaultValue(1)->minimum(0)->maximum(2)
    ->comment("How much time is necessary between kick decision and ball contact [s]")
    ->persisted(true);
  bind->bindNew("time", time, RhIO::Bind::PushOnly)->defaultValue(0.0);
  bind->bindNew("is_kicking", is_kicking, RhIO::Bind::PushOnly)->defaultValue(false)
    ->comment("Is the robot currently kicking?");

  bind->pull();
}

std::string ReactiveKicker::getName() {
  return "ReactiveKicker";
}

void ReactiveKicker::onStart()
{
  // Simply use only classic currently
  expectedKick = "classic";
  is_kicking = false;
  kick_score = 0;
}

void ReactiveKicker::step(float elapsed) {
  // Pull variables from RhIO
  bind->pull();

  time += elapsed;

  // Handle the case where we are currently kicking
  if (is_kicking) {
    // Wait until walk has started and finished kicking
    if (time > 0.25 && !walk->isKicking()) {
      is_kicking = false;
    }
    bind->push();
    return;
  }

  // When is the robot expected to perform the kick
  TimeStamp kick_time = TimeStamp::now()
    + std::chrono::duration<int,std::milli>((int)(anticipation * 1000));

  // First: retrieving ball position in the future 
  LocalisationService * loc = getServices()->localisation;
  Point future_ball_loc = loc->getPredictedBallSelf(kick_time);

  // Simple heuristic, valid for all 'forward' kicks
  kickRight = future_ball_loc.y < 0;

  updateKickScore(elapsed, future_ball_loc);

  // Only use classic
  if (kick_score >= 1.0) {
    requestKick();
    is_kicking = true;
    time = 0;
  }

  bind->push();

}
