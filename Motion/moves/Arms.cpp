#include <math.h>
#include "Arms.h"
#include "services/DecisionService.h"
#include "rhoban_utils/logging/logger.h"
#include "services/ModelService.h"

#include "Kick.h"

static rhoban_utils::Logger logger("arms");

using namespace rhoban_utils;

static double bound(double value, double min, double max)
{
  if (value < min)
    return min;
  if (value > max)
    return max;

  return value;
}

Arms::Arms()
{
  Move::initializeBinding();

  // Arms
  bind->bindNew("armsRoll", armsRoll, RhIO::Bind::PullOnly)->defaultValue(-5.0)->minimum(-20.0)->maximum(150.0);
  bind->bindNew("maintenanceArmsRoll", maintenanceArmsRoll, RhIO::Bind::PullOnly)
      ->defaultValue(40)
      ->minimum(-20.0)
      ->maximum(150.0);
  bind->bindNew("disabledArmsRoll", disabledArmsRoll, RhIO::Bind::PullOnly)
      ->defaultValue(5.0)
      ->minimum(-20.0)
      ->maximum(150.0);

  bind->bindNew("elbowOffset", elbowOffset, RhIO::Bind::PullOnly)->defaultValue(-160.0)->minimum(-200.0)->maximum(30.0);
  bind->bindNew("maintenanceElbowOffset", maintenanceElbowOffset, RhIO::Bind::PullOnly)
      ->defaultValue(-120.0)
      ->minimum(-200.0)
      ->maximum(30.0);
  bind->bindNew("disabledElbowOffset", disabledElbowOffset, RhIO::Bind::PullOnly)
      ->defaultValue(0.0)
      ->minimum(-200.0)
      ->maximum(30.0);
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wstrict-aliasing"
  bind->bindNew("armsState", (int&)armsState, RhIO::Bind::PullOnly)->defaultValue(1);
#pragma GCC diagnostic pop

  bind->bindNew("smoothingArms", smoothingArms, RhIO::Bind::PushOnly)->defaultValue(0);
  armsState = ArmsDisabled;
}

std::string Arms::getName()
{
  return "arms";
}

void Arms::onStart()
{
  bind->pull();
  logger.log("Arms: Starting Move");

  armsState = ArmsEnabled;
  smoothingArms = 0;
}

void Arms::onStop()
{
}

void Arms::step(float elapsed)
{
  ArmsState lastTickArmsState = armsState;
  if (lastTickArmsState != armsState)
    setArms(armsState, true);

  // Update arms

  stepArms(elapsed);
  bind->push();
}

void Arms::setArms(ArmsState newArmsState, bool force, bool init)
{
  if (armsState == newArmsState && !force)
  {
    return;
  }

  lastAngle = actualAngle;

  switch (newArmsState)
  {
    case ArmsEnabled:
    {
      actualAngle.elbow = elbowOffset;
      actualAngle.shoulder_roll = armsRoll;
      actualAngle.shoulder_pitch = rad2deg(getPitch());
      break;
    }
    case ArmsMaintenance:
    {
      actualAngle.elbow = maintenanceElbowOffset;
      actualAngle.shoulder_roll = maintenanceArmsRoll;
      actualAngle.shoulder_pitch = 0;
      break;
    }
    case ArmsDisabled:
    {
      actualAngle.elbow = disabledElbowOffset;
      actualAngle.shoulder_roll = disabledArmsRoll;
      actualAngle.shoulder_pitch = 0;
      break;
    }
  }

  if (init)
    lastAngle = actualAngle;

  armsState = newArmsState;
  bind->node().setInt("armsState", armsState);
  smoothingArms = 0;
}

void Arms::stepArms(double elapsed)
{
  smoothingArms = bound(smoothingArms + elapsed * 3, 0, 1);

  // IMU Pitch to arms
  if (armsState == ArmsEnabled)
    actualAngle.shoulder_pitch = rad2deg(getPitch());

  setAngle("left_shoulder_pitch",
           lastAngle.shoulder_pitch * (1 - smoothingArms) + actualAngle.shoulder_pitch * smoothingArms);
  setAngle("right_shoulder_pitch",
           lastAngle.shoulder_pitch * (1 - smoothingArms) + actualAngle.shoulder_pitch * smoothingArms);

  setAngle("left_shoulder_roll",
           lastAngle.shoulder_roll * (1 - smoothingArms) + actualAngle.shoulder_roll * smoothingArms);
  setAngle("right_shoulder_roll",
           -(lastAngle.shoulder_roll * (1 - smoothingArms) + actualAngle.shoulder_roll * smoothingArms));

  setAngle("left_elbow", lastAngle.elbow * (1 - smoothingArms) + actualAngle.elbow * smoothingArms);
  setAngle("right_elbow", lastAngle.elbow * (1 - smoothingArms) + actualAngle.elbow * smoothingArms);
}
