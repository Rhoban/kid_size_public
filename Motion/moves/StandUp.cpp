#include "StandUp.h"
#include "Arms.h"

#include <rhoban_utils/logging/logger.h>
#include <rhoban_utils/serialization/json_serializable.h>

using rhoban_utils::Function;

static rhoban_utils::Logger logger("StandUp");

StandUp::StandUp(Arms* arms) : arms(arms)
{
  Move::initializeBinding();
  time = 0.0;
  manualT = 0.0;
  trying = 0;
  reloadSpline = false;
  speed = 0.0;

  bind->bindNew("speed_front", speed_front, RhIO::Bind::PullOnly)
      ->defaultValue(1.5)
      ->minimum(0.0)
      ->maximum(20.0)
      ->comment("Speed factor for standing up if the robot is laying on the front.");
  bind->bindNew("speed_back", speed_back, RhIO::Bind::PullOnly)
      ->defaultValue(1.5)
      ->minimum(0.0)
      ->maximum(20.0)
      ->comment("Speed factor for standing up if the robot is laying on the back.");
  bind->bindNew("enable", enable)->defaultValue(true)->persisted(true);
  bind->bindNew("over", over, RhIO::Bind::PushOnly)->comment("Is the move over?")->defaultValue(true);
  bind->bindNew("trying", trying, RhIO::Bind::PushOnly)->comment("Number of try")->defaultValue(0);

  bind->bindNew("delayBefore", delayBefore, RhIO::Bind::PullOnly)
      ->comment("Delay before standing up")
      ->defaultValue(0.5);

  bind->bindNew("armsRoll", armsRoll, RhIO::Bind::PullOnly)->defaultValue(10)->persisted(true)->comment("Arms roll");

  bind->bindNew("useManualT", useManualT, RhIO::Bind::PullOnly)->defaultValue(false)->comment("Use manual time");

  bind->bindNew("manualT", manualT, RhIO::Bind::PullOnly)->defaultValue(0)->comment("Manual T");

  bind->bindNew("layDown", layDown, RhIO::Bind::PullOnly)->defaultValue(false);

  bind->bindNew("reloadSpline", reloadSpline, RhIO::Bind::PushAndPull)->defaultValue(false);
}

StandUp::~StandUp()
{
}

std::string StandUp::getName()
{
  return "standup";
}

void StandUp::onStart()
{
  time = 0.0;
  manualT = 0.0;
  over = false;
  waiting = true;

  arms->setArms(Arms::ArmsState::ArmsDisabled);
}

void StandUp::onStop()
{
  Arms* arms = (Arms*)getMoves()->getMove("arms");
  arms->setArms(Arms::ArmsState::ArmsEnabled);
}

void StandUp::setLayDown(bool value)
{
  bind->node().setBool("layDown", value);
}

void StandUp::step(float elapsed)
{
  bind->pull();

  // Be sure walk arms are disabled

  if (reloadSpline)
  {
    splines = Function::fromFile(currentSpline);
    reloadSpline = false;
  }

  if (waiting)
  {
    time += elapsed;
    if (useManualT || time > delayBefore)
    {
      try
      {
        if (layDown)
        {
          splines = Function::fromFile("standup_front.json");
          layDownEnd = 0;
          for (layDownEnd = 0; layDownEnd < 10; layDownEnd += 0.1)
          {
            if (splines["over"].get(layDownEnd) > 0.5)
            {
              break;
            }
          }
          layDownEnd += 1;
          speed = 1;
        }
        else
        {
          if (getPitch() < 0)
          {
            currentSpline = "standup_back.json";
            splines = Function::fromFile("standup_back.json");
            speed = speed_back;
          }
          else
          {
            currentSpline = "standup_front.json";
            splines = Function::fromFile("standup_front.json");
            speed = speed_front;
          }
        }
      }
      catch (const rhoban_utils::JsonParsingError& exc)
      {
        logger.error("%s", exc.what());
      }

      waiting = false;
      time = 0;
    }
  }
  else
  {
    if (enable)
    {
      float finalSpeed = speed / (1 + trying / 2.0);

      if (finalSpeed < 0.5)
        finalSpeed = 0.5;
      double remap = splines["remap"].get(time);
      // std::cout << "Remap=" << remap << ", speed=" << finalSpeed << ", elapsed: " << elapsed << std::endl;
      time += elapsed * remap * finalSpeed;
      if (useManualT)
      {
        time = manualT;
      }
      else
      {
        manualT = time;
      }

      double playTime = time;
      if (layDown)
      {
        // Playing the spline backward to lay down
        playTime = layDownEnd - playTime;
      }

      setAngle("left_shoulder_roll", armsRoll);
      setAngle("right_shoulder_roll", -armsRoll);
      setAngle("left_shoulder_pitch", splines["shoulder_pitch"].get(playTime));
      setAngle("right_shoulder_pitch", splines["shoulder_pitch"].get(playTime));
      setAngle("left_elbow", splines["elbow"].get(playTime));
      setAngle("right_elbow", splines["elbow"].get(playTime));
      setAngle("left_hip_pitch", splines["hip_pitch"].get(playTime));
      setAngle("right_hip_pitch", splines["hip_pitch"].get(playTime));
      setAngle("left_knee", splines["knee"].get(playTime));
      setAngle("right_knee", splines["knee"].get(playTime));
      setAngle("left_ankle_pitch", splines["ankle_pitch"].get(playTime));
      setAngle("right_ankle_pitch", splines["ankle_pitch"].get(playTime));
      if (layDown)
      {
        over = playTime < 0;
      }
      else
      {
        over = splines["over"].get(playTime) > 0.5;
      }

      if (useManualT)
      {
        over = false;
      }
    }
    else
    {
      over = true;
    }
  }

  bind->push();
}
