#include <math.h>
#include "WalkTest.h"
#include "services/ModelService.h"
#include "rhoban_utils/angle.h"
#include <rhoban_utils/control/variation_bound.h>

using namespace rhoban_utils;

WalkTest::WalkTest()
{
  Move::initializeBinding();
  swingGainStart = 0.04;

  // Enables or disables the walk
  bind->bindNew("walkEnable", walkEnable, RhIO::Bind::PullOnly)->defaultValue(false);
  bind->bindNew("walkStep", walkStep, RhIO::Bind::PullOnly)->comment("Walk control Step [mm/step]")->defaultValue(0.0);
  bind->bindNew("walkLateral", walkLateral, RhIO::Bind::PullOnly)
      ->comment("Walk control Lateral [mm/step]")
      ->defaultValue(0.0);
  bind->bindNew("walkTurn", walkTurn, RhIO::Bind::PullOnly)->comment("Walk control Turn [deg/step]")->defaultValue(0.0);

  // Walk engine parameters
  bind->bindNew("trunkXOffset", engine.trunkXOffset, RhIO::Bind::PullOnly)->defaultValue(engine.trunkXOffset);
  bind->bindNew("trunkZOffset", engine.trunkZOffset, RhIO::Bind::PullOnly)->defaultValue(engine.trunkZOffset);
  bind->bindNew("frequency", engine.frequency, RhIO::Bind::PullOnly)->defaultValue(engine.frequency);
  bind->bindNew("footYOffset", engine.footYOffset, RhIO::Bind::PullOnly)->defaultValue(engine.footYOffset);
  bind->bindNew("riseGain", engine.riseGain, RhIO::Bind::PullOnly)->defaultValue(engine.riseGain);
  bind->bindNew("riseDuration", engine.riseDuration, RhIO::Bind::PullOnly)->defaultValue(engine.riseDuration);
  bind->bindNew("swingGain", engine.swingGain, RhIO::Bind::PullOnly)->defaultValue(engine.swingGain);
  bind->bindNew("swingGainStart", swingGainStart, RhIO::Bind::PullOnly)->defaultValue(swingGainStart);
  bind->bindNew("swingPhase", engine.swingPhase, RhIO::Bind::PullOnly)->defaultValue(engine.swingPhase);
  bind->bindNew("footYOffsetPerYSpeed", engine.footYOffsetPerYSpeed, RhIO::Bind::PullOnly)
      ->defaultValue(engine.footYOffsetPerYSpeed);
  bind->bindNew("trunkPitch", trunkPitch, RhIO::Bind::PullOnly)->defaultValue(-15);

  // Acceleration limits
  bind->bindNew("maxDStepByCycle", maxDStepByCycle, RhIO::Bind::PullOnly)
      ->defaultValue(30)
      ->comment("Maximal difference between two steps [mm/step^2]");
  bind->bindNew("maxDLatByCycle", maxDLatByCycle, RhIO::Bind::PullOnly)
      ->defaultValue(20)
      ->comment("Maximal difference between two steps [mm/step^2]");
  bind->bindNew("maxDTurnByCycle", maxDTurnByCycle, RhIO::Bind::PullOnly)
      ->defaultValue(10)
      ->comment("Maximal difference between two steps [deg/step^2]");

  // Security parameters
  bind->bindNew("securityThreshold", securityThreshold)->defaultValue(0.065)->minimum(0)->maximum(1.0);
  bind->bindNew("securityPhase", securityPhase)->defaultValue(0.1);

  // Arms
  bind->bindNew("armsRoll", armsRoll, RhIO::Bind::PullOnly)
      ->defaultValue(-5.0)
      ->minimum(-20.0)
      ->maximum(150.0)
      ->persisted(true);
  bind->bindNew("elbowOffset", elbowOffset, RhIO::Bind::PullOnly)
      ->defaultValue(-168.0)
      ->minimum(-200.0)
      ->maximum(30.0)
      ->persisted(true);

  timeSinceLastStep = 0;
}

std::string WalkTest::getName()
{
  return "walk_test";
}

void WalkTest::onStart()
{
  Leph::HumanoidFixedModel& model = getServices()->model->goalModel();
  engine.initByModel(model);

  bind->node().setBool("walkEnable", false);

  // Zeroing the orders
  bind->node().setFloat("walkStep", 0.0);
  bind->node().setFloat("walkLateral", 0.0);
  bind->node().setFloat("walkTurn", 0.0);

  state = WalkNotWalking;
}

void WalkTest::step(float elapsed)
{
  bind->pull();

  if (state == WalkNotWalking)
  {
    // Walk is not enabled, just freezing the engine
    engine.riseGain = 0;
    engine.swingGain = 0;
    engine.xSpeed = 0;
    engine.ySpeed = 0;
    engine.yawSpeed = 0;
    engine.reset();
    engine.update(0);
    timeSinceLastStep = 0;
    stepCount = 0;

    if (walkEnable)
    {
      state = WalkStarting;
    }
  }
  else
  {
    // Ticking
    double prevPhase = engine.getStepPhase();
    double over = engine.update(timeSinceLastStep + elapsed);
    bool securityBlock = false;

    // Doing security check
    if (prevPhase < securityPhase && engine.getStepPhase() > securityPhase)
    {
      double pressureY = getPressureY();

      if (state == Walking) {
        if (engine.isLeftSupport && (pressureY < -securityThreshold) ||
            !engine.isLeftSupport && (pressureY > securityThreshold))
        {
          securityBlock = true;
          engine.update(timeSinceLastStep);
        }
      }
    }

    if (!securityBlock)
    {
      timeSinceLastStep += elapsed;

      // New step condition
      if (over > 0 || state == WalkStarting)
      {
        timeSinceLastStep = over;
        stepCount += 1;

        if (stepCount <= 2)
        {
          // We apply an extra swing to start safely to walk
          engine.swingGain = swingGainStart;
        }

        if (state != WalkStarting)
        {
          if (walkEnable)
          {
            // We should just walk normally
            state = Walking;
          }
          else
          {
            bool walkingTooMuch =
                fabs(engine.xSpeed) > 0.01 || fabs(engine.ySpeed) > 0.01 || rad2deg(fabs(engine.yawSpeed)) > 3;
            if (state == Walking && walkingTooMuch)
            {
              // We will apply an extra step with null orders to be sure the walk stops properly
              state = WalkStopping;
            }
            else
            {
              // We stop walking now
              state = WalkNotWalking;
            }
          }
        }

        if (state != Walking)
        {
          // We are not walking, starting or starting, we have no orders
          engine.xSpeed = 0;
          engine.ySpeed = 0;
          engine.yawSpeed = 0;
        }
        else
        {
          // Updating engine speed according to acc. limits
          VariationBound::update(engine.xSpeed, walkStep / 1000.0, maxDStepByCycle / 1000.0, 1);
          VariationBound::update(engine.ySpeed, walkLateral / 1000.0, maxDLatByCycle / 1000.0, 1);
          VariationBound::update(engine.yawSpeed, deg2rad(walkTurn), deg2rad(maxDTurnByCycle), 1);
        }

        if (state == WalkStarting)
        {
          std::cout << "Going to WALKING" << std::endl;
          state = Walking;
        }

        // Creating a new footstep
        engine.newStep();

        // Updating the engine again with the time elapsed since it began
        engine.update(timeSinceLastStep);
      }
    }
  }

  // Assigning to robot
  engine.assignModel(getServices()->model->goalModel());

  // Flushing engine leg orders to robot
  ModelService* model = getServices()->model;
  model->flushLegs(_smoothing);

  // Applying extra trunk pitch
  setAngle("left_hip_pitch", trunkPitch);
  setAngle("right_hip_pitch", trunkPitch);

  // Update arms
  stepArms();

  bind->push();
}

void WalkTest::stepArms()
{
  // IMU Pitch to arms
  float imuPitch = rad2deg(getPitch());
  setAngle("left_shoulder_pitch", imuPitch);
  setAngle("right_shoulder_pitch", imuPitch);
  // Rolls to arms
  setAngle("left_shoulder_roll", armsRoll);
  setAngle("right_shoulder_roll", -armsRoll);
  // Elbows
  setAngle("left_elbow", elbowOffset);
  setAngle("right_elbow", elbowOffset);
}
