#include "ThrowIn.h"

#include <rhoban_utils/logging/logger.h>
#include <rhoban_utils/serialization/json_serializable.h>

using rhoban_utils::Function;

static rhoban_utils::Logger logger("ThrowIn");

ThrowIn::ThrowIn()
{
  Move::initializeBinding();
  bind->bindNew("over", over, RhIO::Bind::PushOnly)->defaultValue(false)->comment("Is the kick over");
  bind->bindNew("useManualT", useManualT, RhIO::Bind::PullOnly)->defaultValue(false)->comment("Use manual time");
  bind->bindNew("manualT", manualT, RhIO::Bind::PullOnly)->defaultValue(0)->comment("Manual T");

  bind->bindNew("reloadSpline", reloadSpline, RhIO::Bind::PushAndPull)->defaultValue(false);
}

ThrowIn::~ThrowIn()
{
}

std::string ThrowIn::getName()
{
  return "throwin";
}

void ThrowIn::onStart()
{
  // init
  over = false;
  time = 0.0;
  manualT = 0.0;
  trying = 0;
  reloadSpline = true;

  // loading splines
  try
  {
    splines = Function::fromFile("kick_throwin.json");
  }
  catch (const rhoban_utils::JsonParsingError& exc)
  {
    logger.error("%s", exc.what());
  }

  // looking for tMax in remap
  tMax = 0;
  for (auto& entry : splines)
  {
    double duration = entry.second.getXMax();
    if (duration > tMax)
      tMax = duration;
  }
}

void ThrowIn::onStop()
{
  setAngle("left_shoulder_roll", 0);
  setAngle("right_shoulder_roll", 0);
}

void ThrowIn::step(float elapsed)
{
  bind->pull();

  if (reloadSpline)
  {
    // loading splines
    try
    {
      splines = Function::fromFile("kick_throwin.json");
    }
    catch (const rhoban_utils::JsonParsingError& exc)
    {
      logger.error("%s", exc.what());
    }

    // looking for tMax in remap
    tMax = 0;
    for (auto& entry : splines)
    {
      double duration = entry.second.getXMax();
      if (duration > tMax)
        tMax = duration;
    }

    reloadSpline = false;
  }

  time += elapsed * splines["remap"].get(time);
  if (useManualT)
  {
    time = manualT;
  }
  else
  {
    manualT = time;
  }

  if (time > tMax)
  {
    this->Move::stop();
  }

  else
  {
    setAngle("left_hip_pitch", splines["hip_pitch"].get(time));
    setAngle("right_hip_pitch", splines["hip_pitch"].get(time));
    setAngle("left_knee", splines["knee"].get(time));
    setAngle("right_knee", splines["knee"].get(time));
    setAngle("left_ankle_pitch", splines["ankle_pitch"].get(time));
    setAngle("right_ankle_pitch", splines["ankle_pitch"].get(time));
    setAngle("left_shoulder_roll", splines["left_shoulder_roll"].get(time));
    setAngle("right_shoulder_roll", splines["right_shoulder_roll"].get(time));
    setAngle("left_shoulder_pitch", splines["shoulder_pitch"].get(time));
    setAngle("right_shoulder_pitch", splines["shoulder_pitch"].get(time));
    setAngle("left_elbow", splines["elbow"].get(time));
    setAngle("right_elbow", splines["elbow"].get(time));
    over = splines["over"].get(time) > 0.5;
    if (useManualT)
    {
      over = false;
    }
  }

  bind->push();
}
