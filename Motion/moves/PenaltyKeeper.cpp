#include "PenaltyKeeper.h"

#include <services/LocalisationService.h>
#include <services/DecisionService.h>
#include <robocup_referee/constants.h>

#include <rhoban_utils/logging/logger.h>

#include "Head.h"
#include "Walk.h"

using namespace rhoban_geometry;
using namespace rhoban_utils;

using robocup_referee::Constants;

static Logger logger("penalty_keeper");

PenaltyKeeper::PenaltyKeeper(Walk* walk, Head* head)
  : walk(walk), head(head), t(0), is_running(false), is_diving(false), dive_left(false)
{
  Move::initializeBinding();

  bind->bindNew("checkOnly", check_only, RhIO::Bind::PullOnly)->comment("Is robot faking dive")->defaultValue(false);
  bind->bindNew("t", t, RhIO::Bind::PushOnly)->comment("time since last change of state");
  bind->bindNew("isRunning", is_running, RhIO::Bind::PushOnly)
      ->comment("Is robot prepared to dive")
      ->defaultValue(false);
  bind->bindNew("isDiving", is_diving, RhIO::Bind::PushOnly)
      ->comment("Is robot performing diving motion")
      ->defaultValue(false);
  bind->bindNew("diveLeft", dive_left, RhIO::Bind::PushOnly)
      ->comment("Is robot diving to the left")
      ->defaultValue(true);

  bind->bindNew("min_dive_y", min_dive_y, RhIO::Bind::PullOnly)
      ->comment("Minimal distance to center when intercepting to dive on one side [m]")
      ->defaultValue(0.2);
  bind->bindNew("em_start", em_start, RhIO::Bind::PullOnly)
      ->comment("Time between start of the dive and release of the motors [s]")
      ->defaultValue(0.5);
  bind->bindNew("anticipation", anticipation, RhIO::Bind::PullOnly)
      ->comment("Time between start of the dive and expected interception of the ball [s]")
      ->defaultValue(1.0);

  bind->pull();
}

std::string PenaltyKeeper::getName()
{
  return "penalty_keeper";
}

void PenaltyKeeper::onStart()
{
  is_running = false;
  is_diving = false;
  bind->pull();

  RhIO::Root.setFloat("/moves/walk/trunkZOffset", 0.12);
  RhIO::Root.setFloat("/moves/walk/trunkPitch", 8);
  RhIO::Root.setInt("/moves/arms/armsState", 0);
  RhIO::Root.setFloat("/moves/walk/footYOffset", 0.03);
  // Avoid looking to high and get start of motion more easily
  RhIO::Root.setFloat("/moves/head/minTilt", 0.0);
  RhIO::Root.setFloat("/moves/head/maxPan", 40.0);
  head->setForceTrack(true);
  head->setDisabled(true);
}

void PenaltyKeeper::onStop()
{
}

void PenaltyKeeper::step(float elapsed)
{
  // Pull variables from RhIO
  bind->pull();

  LocalisationService* localisation = getServices()->localisation;
  DecisionService* decision = getServices()->decision;

  t += elapsed;

  if (decision->isBallQualityGood)
    localisation->customBallReset(2, -2);

  // Checking status
  if (is_running && (decision->handled || decision->isFallen))
  {
    is_running = false;
    head->setDisabled(true);
  }
  if (!is_running && !decision->handled && !decision->isFallen)
  {
    t = 0;
    localisation->customFieldReset(-Constants::field.field_length / 2, 0, 0.1, 0, 0.1);
    // localisation->customBallReset(Constants::field.penalty_mark_dist, 0);
    localisation->customBallReset(1, -3);
    is_running = true;
    head->setDisabled(false);
  }

  // Checking future position of the ball
  TimeStamp interception_time = TimeStamp::now().addMS(anticipation * 1000);
  Point future_ball_loc = localisation->getPredictedBallSelf(interception_time, true);

  // TODO: note: intersection point with axis should be used rather than predicted pos

  logger.log("Future ball pos self %lf, %lf", future_ball_loc.x, future_ball_loc.y);

  // Checking if we should start diving
  if (is_running && !is_diving && decision->isBallQualityGood)
  {
    is_diving = future_ball_loc.x < 0 && std::fabs(future_ball_loc.y) > min_dive_y;
    dive_left = future_ball_loc.y > 0;
    t = 0;
    setShouldersTorque(true);
    head->setDisabled(false);
    head->setForceTrack(true);
  }
  if (is_diving && t > 3)
  {
    is_diving = false;
    is_running = false;
  }

  if (!check_only)
  {
    setAngle("left_shoulder_pitch", -160);
    setAngle("right_shoulder_pitch", -160);
  }

  // Applying dive
  if (is_diving)
  {
    head->setDisabled(true);
    std::string dive_side = dive_left ? "left" : "right";
    std::string opp_side = dive_left ? "right" : "left";
    int sign = dive_left ? 1 : -1;
    if (check_only)
    {
      setAngle(dive_side + "_shoulder_roll", sign * 90);
    }
    else
    {
      double ratio = std::min(1.0, t / em_start);
      setAngle(opp_side + "_knee", -80 * ratio);
      setAngle(opp_side + "_ankle_pitch", 40 * ratio);
      setAngle(opp_side + "_hip_pitch", 40 * ratio);
      setAngle(dive_side + "_shoulder_roll", -20 * sign * ratio);
      // Not very satisfying
      // if (t >= em_start)
      //  setShouldersTorque(false);
    }
  }

  bind->push();
}

void PenaltyKeeper::setShouldersTorque(bool enabled)
{
  for (const std::string& side : { "left", "right" })
  {
    for (const std::string& motor : { "_shoulder_roll", "_shoulder_pitch" })
    {
      setTorqueLimit(side + motor, enabled);
    }
  }
}
