#include "VisionLogMachine.hpp"
#include "Head.h"
#include "Placer.h"
#include "Walk.h"

#include <scheduler/MoveScheduler.h>
#include <services/LocalisationService.h>

#include <rhoban_geometry/point.h>
#include <rhoban_random/tools.h>
#include <rhoban_utils/logging/logger.h>
#include <robocup_referee/constants.h>

#define STATE_INITIAL "Initial"
#define STATE_MOVING "Moving"
#define STATE_SCAN "Scan"
#define STATE_ENDED "Ended"

static rhoban_utils::Logger logger("VisionLogMachine");

using namespace robocup_referee;
using namespace rhoban_utils;
using namespace rhoban_geometry;

VisionLogMachine::VisionLogMachine(Walk* walk_, Head* head_, Placer* placer_)
  : walk(walk_), head(head_), placer(placer_)
{
  engine = rhoban_random::getRandomEngine();
  setState(STATE_INITIAL);
  Move::initializeBinding();
  // RhIO binding variables
  bind->bindNew("state", state, RhIO::Bind::PushOnly)->comment("State of the VisionLogMachine STM");
  bind->bindNew("runDuration", runDuration, RhIO::Bind::PullOnly)->defaultValue(100)->comment("Duration of a run [s]");
  bind->bindNew("runTime", runTime, RhIO::Bind::PushOnly)
      ->defaultValue(0)
      ->comment("Time elapsed since the start of the motion [s]");
  bind->bindNew("stateTime", stateTime, RhIO::Bind::PushOnly)
      ->defaultValue(0)
      ->comment("Time elapsed in current state [s]");
  bind->bindNew("minX", minX, RhIO::Bind::PullOnly)
      ->defaultValue(-Constants::field.field_length / 2)
      ->comment("Minimal target on the field according to x-axis [m]");
  bind->bindNew("maxX", maxX, RhIO::Bind::PullOnly)
      ->defaultValue(Constants::field.field_length / 2)
      ->comment("Maximal target on the field according to x-axis [m]");
  bind->bindNew("minY", minY, RhIO::Bind::PullOnly)
      ->defaultValue(-Constants::field.field_width / 2)
      ->comment("Minimal target on the field according to y-axis [m]");
  bind->bindNew("maxY", maxY, RhIO::Bind::PullOnly)
      ->defaultValue(Constants::field.field_width / 2)
      ->comment("Maximal target on the field according to y-axis [m]");
  bind->bindNew("minDistance", minDistance, RhIO::Bind::PullOnly)
      ->defaultValue(0.5)
      ->comment("Minimal distance between two scanning points [m]");
  bind->bindNew("maxDistance", maxDistance, RhIO::Bind::PullOnly)
      ->defaultValue(1.5)
      ->comment("Maximal distance between two scanning points [m]");
}

std::string VisionLogMachine::getName()
{
  return "vision_log_machine";
}

void VisionLogMachine::onStart()
{
  setState(STATE_INITIAL);
  // Ensure that walk and head are properly running
  walk->start();
  head->start();
  runTime = 0;
}

void VisionLogMachine::onStop()
{
  setState(STATE_ENDED);
}

void VisionLogMachine::step(float elapsed)
{
  bind->pull();
  runTime += elapsed;
  stateTime += elapsed;

  if (runTime >= runDuration)
  {
    stop();
  }

  if (state == STATE_INITIAL)
  {
    setState(STATE_MOVING);
  }
  else if (state == STATE_SCAN)
  {
    if (stateTime >= head->getScanPeriod())
    {
      setState(STATE_MOVING);
    }
  }
  else if (state == STATE_MOVING)
  {
    if (placer->arrived)
    {
      setState(STATE_SCAN);
    }
  }

  bind->push();
}

void VisionLogMachine::enterState(std::string state)
{
  stateTime = 0;
  if (state == STATE_SCAN)
  {
    head->setDisabled(false);
    head->setForceScanBall(true);
  }
  else
  {
    head->setDisabled(true);
  }

  if (state == STATE_MOVING)
  {
    placer->start();
    generatePlacerOrder();
  }
  else
  {
    placer->stop();
  }
}

void VisionLogMachine::exitState(std::string state)
{
}

void VisionLogMachine::generatePlacerOrder()
{
  LocalisationService* loc = getServices()->localisation;

  rhoban_geometry::Point current_pos = loc->getFieldPos();
  bool valid_successor = false;

  rhoban_geometry::Point next_pos;
  int nb_trials = 0;
  int max_trials = 100;
  while (!valid_successor && nb_trials < max_trials)
  {
    next_pos = current_pos + rhoban_geometry::Point::mkRandomPolar(minDistance, maxDistance, &engine);
    valid_successor = next_pos.x >= minX && next_pos.x <= maxX && next_pos.y >= minY && next_pos.y <= maxY;
    nb_trials++;
  }
  if (!valid_successor)
  {
    logger.log("Failed to find a valid successor after %d iterations, going to the center of the area", nb_trials);
    next_pos.x = (minX + maxX) / 2;
    next_pos.y = (minY + maxY) / 2;
  }
  std::uniform_real_distribution<float> azimuth_distrib(0, 360);
  float next_azimuth = azimuth_distrib(engine);
  placer->goTo(next_pos.x, next_pos.y, next_azimuth);
}
