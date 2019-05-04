#include <math.h>
#include "Walk.h"
#include "services/DecisionService.h"
#include "services/ModelService.h"
#include "rhoban_utils/angle.h"
#include <rhoban_utils/logging/logger.h>
#include <rhoban_utils/control/variation_bound.h>
#include "Kick.h"

static rhoban_utils::Logger walkLogger("Walk");

using namespace rhoban_utils;

static double bound(double value, double min, double max)
{
  if (value < min)
    return min;
  if (value > max)
    return max;

  return value;
}

Walk::Walk(Kick* _kickMove) : kickMove(_kickMove)
{
  Move::initializeBinding();

  // Enables or disables the walk
  bind->bindNew("walkEnable", walkEnable, RhIO::Bind::PullOnly)->defaultValue(false);
  bind->bindNew("walkStep", walkStep, RhIO::Bind::PullOnly)->comment("Walk control Step [mm/step]")->defaultValue(0.0);
  bind->bindNew("walkLateral", walkLateral, RhIO::Bind::PullOnly)
      ->comment("Walk control Lateral [mm/step]")
      ->defaultValue(0.0);
  bind->bindNew("walkTurn", walkTurn, RhIO::Bind::PullOnly)->comment("Walk control Turn [deg/step]")->defaultValue(0.0);

  // Walk limits (to inform other moves about limits)
  bind->bindNew("maxRotation", maxRotation, RhIO::Bind::PullOnly)->defaultValue(15.0);
  bind->bindNew("maxStep", maxStep, RhIO::Bind::PullOnly)->defaultValue(65.0);
  bind->bindNew("maxStepBackward", maxStepBackward, RhIO::Bind::PullOnly)->defaultValue(35.0);
  bind->bindNew("maxLateral", maxLateral, RhIO::Bind::PullOnly)->defaultValue(30.0);

  // Walk engine parameters
  bind->bindNew("trunkXOffset", engine.trunkXOffset, RhIO::Bind::PullOnly)->defaultValue(engine.trunkXOffset);
  bind->bindNew("trunkZOffset", engine.trunkZOffset, RhIO::Bind::PullOnly)->defaultValue(engine.trunkZOffset);
  bind->bindNew("frequency", engine.frequency, RhIO::Bind::PullOnly)->defaultValue(engine.frequency);
  bind->bindNew("footYOffset", engine.footYOffset, RhIO::Bind::PullOnly)->defaultValue(engine.footYOffset);
  bind->bindNew("riseGain", engine.riseGain, RhIO::Bind::PullOnly)->defaultValue(engine.riseGain);
  bind->bindNew("riseDuration", engine.riseDuration, RhIO::Bind::PullOnly)->defaultValue(engine.riseDuration);
  bind->bindNew("swingGain", engine.swingGain, RhIO::Bind::PullOnly)->defaultValue(engine.swingGain);
  bind->bindNew("swingGainStart", swingGainStart, RhIO::Bind::PullOnly)->defaultValue(swingGainStart = 0.04);
  bind->bindNew("swingPhase", engine.swingPhase, RhIO::Bind::PullOnly)->defaultValue(engine.swingPhase);
  bind->bindNew("footYOffsetPerYSpeed", engine.footYOffsetPerYSpeed, RhIO::Bind::PullOnly)
      ->defaultValue(engine.footYOffsetPerYSpeed);
  bind->bindNew("trunkPitch", trunkPitch, RhIO::Bind::PullOnly)->defaultValue(trunkPitch = 13);
  bind->bindNew("bootstrapSteps", bootstrapSteps, RhIO::Bind::PullOnly)->defaultValue(bootstrapSteps = 3);
  bind->bindNew("shouldBootstrap", shouldBootstrap, RhIO::Bind::PullOnly)->defaultValue(shouldBootstrap = false);

  // Acceleration limits
  bind->bindNew("maxDStepByCycle", maxDStepByCycle, RhIO::Bind::PullOnly)
      ->defaultValue(20)
      ->comment("Maximal difference between two steps [mm/step^2]");
  bind->bindNew("maxDLatByCycle", maxDLatByCycle, RhIO::Bind::PullOnly)
      ->defaultValue(20)
      ->comment("Maximal difference between two steps [mm/step^2]");
  bind->bindNew("maxDTurnByCycle", maxDTurnByCycle, RhIO::Bind::PullOnly)
      ->defaultValue(10)
      ->comment("Maximal difference between two steps [deg/step^2]");

  // Security parameters
  bind->bindNew("securityThreshold", securityThreshold, RhIO::Bind::PullOnly)
      ->defaultValue(0.065)
      ->minimum(0)
      ->maximum(1.0);
  bind->bindNew("securityPhase", securityPhase, RhIO::Bind::PullOnly)->defaultValue(0.05);

  // Kick parameters
  bind->bindNew("kickPending", kickPending, RhIO::Bind::PullOnly)->defaultValue(false);
  bind->bindNew("kickLeftFoot", kickLeftFoot, RhIO::Bind::PullOnly)->defaultValue(false);
  bind->bindNew("kickName", kickName, RhIO::Bind::PullOnly)->defaultValue("classic");
  bind->bindNew("kickCooldown", kickCooldown, RhIO::Bind::PullOnly)
      ->defaultValue(0.5)
      ->comment("Cooldown duration [s]");
  bind->bindNew("kickWarmup", kickWarmup, RhIO::Bind::PullOnly)->defaultValue(0.75)->comment("Warmup [s]");

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

  state = WalkNotWalking;
  kickState = KickNotKicking;
  timeSinceLastStep = 0;
}

std::string Walk::getName()
{
  return "walk";
}

void Walk::onStart()
{
  Leph::HumanoidFixedModel& model = getServices()->model->goalModel();
  engine.initByModel(model);

  bind->node().setBool("walkEnable", false);

  // Zeroing the orders
  bind->node().setFloat("walkStep", 0.0);
  bind->node().setFloat("walkLateral", 0.0);
  bind->node().setFloat("walkTurn", 0.0);

  // Cancelling eventual previous pending kicks
  bind->node().setBool("kickPending", false);

  state = WalkNotWalking;
  kickState = KickNotKicking;
  armsEnabled = true;
  smoothingArms = 0;
}

void Walk::onStop()
{
  Helpers::getServices()->model->setReadBaseUpdate(false);
  state = WalkNotWalking;
  kickState = KickNotKicking;
}

void Walk::control(bool enable, double step, double lateral, double turn)
{
  if (isRunning())
  {
    bind->node().setBool("walkEnable", enable);
    bind->node().setFloat("walkStep", bound(step, -maxStepBackward, maxStep));
    bind->node().setFloat("walkLateral", bound(lateral, -maxLateral, maxLateral));
    bind->node().setFloat("walkTurn", bound(turn, -maxRotation, maxRotation));
  }
}

void Walk::kick(bool rightFoot, const std::string& kickName)
{
  walkLogger.log("Kick \"%s\" requested", kickName.c_str());

  bind->node().setBool("kickPending", true);
  bind->node().setBool("kickLeftFoot", !rightFoot);
  bind->node().setStr("kickName", kickName);
}

bool Walk::isKicking()
{
  return kickState != KickNotKicking;
}

void Walk::setShouldBootstrap(bool bootstrap)
{
  bind->node().setBool("shouldBootstrap", bootstrap);
}

void Walk::step(float elapsed)
{
  bind->pull();
  engine.trunkPitch = deg2rad(trunkPitch);

  stepKick(elapsed);

  getServices()->model->setReadBaseUpdate(state != WalkNotWalking);

  if (state == WalkNotWalking)
  {
    // Walk is not enabled, just freezing the engine
    engine.riseGain = 0;
    engine.swingGain = 0;
    engine.xSpeed = 0;
    engine.ySpeed = 0;
    engine.yawSpeed = 0;
    engine.reset();
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
    double prevPhase = timeSinceLastStep / engine.stepDuration;
    double phase = (timeSinceLastStep + elapsed) / engine.stepDuration;
    bool securityBlock = false;

    // Doing security check
    if (prevPhase < securityPhase && phase > securityPhase)
    {
      double pressureY = getPressureY();

      if (state == Walking)
      {
        // Checking that the center of pressure is not on the opposite foot
        if ((engine.isLeftSupport && (pressureY < -securityThreshold)) ||
            (!engine.isLeftSupport && (pressureY > securityThreshold)))
        {
          securityBlock = true;
        }
      }
    }

    if (!securityBlock)
    {
      timeSinceLastStep += elapsed;

      // New step condition
      if (timeSinceLastStep > engine.stepDuration || state == WalkStarting)
      {
        timeSinceLastStep -= engine.stepDuration;
        stepCount += 1;

        if (stepCount <= 2)
        {
          // We apply an extra swing to start safely to walk
          engine.swingGain = swingGainStart;
        }

        if (state != WalkStarting && state != WalkBootstrapingSteps)
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
          if (shouldBootstrap)
          {
            state = WalkBootstrapingSteps;
          }
          else
          {
            state = Walking;
          }
        }

        if (state == WalkBootstrapingSteps && stepCount > bootstrapSteps)
        {
          setShouldBootstrap(false);
          state = Walking;
        }

        // Creating a new footstep
        engine.newStep();

        if (engine.isLeftSupport)
        {
          getServices()->model->goalModel().setSupportFoot(Leph::HumanoidFixedModel::LeftSupportFoot);
        }
        else
        {
          getServices()->model->goalModel().setSupportFoot(Leph::HumanoidFixedModel::RightSupportFoot);
        }
      }
    }
  }

  // Assigning to robot
  engine.assignModel(getServices()->model->goalModel(), timeSinceLastStep);

  // Flushing engine leg orders to robot
  ModelService* model = getServices()->model;
  model->flushLegs(_smoothing);

  // Update arms
  stepArms(elapsed);

  bind->push();
}

void Walk::stepKick(float elapsed)
{
  if (kickPending)
  {
    // Enter the kick STM
    bind->node().setBool("kickPending", false);
    kickState = KickWaitingWalkToStop;
  }

  if (kickState != KickNotKicking)
  {
    // While kicking, walk is forced isabled
    walkEnable = false;
    kickT += elapsed;

    if (kickState == KickWaitingWalkToStop && state == WalkNotWalking)
    {
      // Walk is over, go to warmup state
      kickState = KickWarmup;
      kickT = 0;
    }
    DecisionService* decision = getServices()->decision;

    if (kickState == KickWarmup && kickT >= kickWarmup && !decision->freezeKick)
    {
      // Warmup over, start the kick move
      kickState = KickKicking;
      kickMove->set(kickLeftFoot, kickName);
      startMove("kick", 0.0);
    }
    if (kickState == KickKicking && kickMove->over)
    {
      // Kick is over, enter the coolDown sequence
      kickState = KickCooldown;
      kickT = 0;
    }
    if (kickState == KickCooldown && kickT >= kickCooldown)
    {
      // Cooldown is over, quitting kicking state
      kickState = KickNotKicking;
    }
  }
}

void Walk::enableArms(bool enabled)
{
  armsEnabled = enabled;
}

void Walk::stepArms(double elapsed)
{
  smoothingArms = bound(smoothingArms + elapsed * (armsEnabled ? 3 : -3), 0, 1);

  // IMU Pitch to arms
  float imuPitch = rad2deg(getPitch());
  setAngle("left_shoulder_pitch", imuPitch * smoothingArms);
  setAngle("right_shoulder_pitch", imuPitch * smoothingArms);

  // Rolls to arms
  setAngle("left_shoulder_roll", armsRoll * smoothingArms);
  setAngle("right_shoulder_roll", -armsRoll * smoothingArms);

  // Elbows
  setAngle("left_elbow", elbowOffset * smoothingArms);
  setAngle("right_elbow", elbowOffset * smoothingArms);
}

bool Walk::isNewStep(double elapsed)
{
  return timeSinceLastStep + elapsed > engine.stepDuration;
}

double Walk::getPhase()
{
  double stepHalfPhase = bound(timeSinceLastStep / engine.stepDuration, 0, 1) * 0.5;

  if (engine.isLeftSupport)
  {
    return stepHalfPhase;
  }
  else
  {
    return 0.5 + stepHalfPhase;
  }
}

Eigen::Vector3d Walk::getMinOrders() const
{
  Eigen::Vector3d bound;
  bound << -maxStepBackward / 1000, -maxLateral / 1000, deg2rad(-maxRotation);
  return bound;
}

Eigen::Vector3d Walk::getMaxOrders() const
{
  Eigen::Vector3d bound;
  bound << maxStep / 1000, maxLateral / 1000, deg2rad(maxRotation);
  return bound;
}

Eigen::Vector3d Walk::getMinDeltaOrders() const
{
  Eigen::Vector3d bound;
  bound << -maxDStepByCycle / 1000, -maxDLatByCycle / 1000, deg2rad(-maxDTurnByCycle);
  return bound;
}

Eigen::Vector3d Walk::getMaxDeltaOrders() const
{
  Eigen::Vector3d bound;
  bound << maxDStepByCycle / 1000, maxDLatByCycle / 1000, deg2rad(maxDTurnByCycle);
  return bound;
}

Eigen::Vector4d Walk::getRawOrder() const
{
  return Eigen::Vector4d(engine.xSpeed, engine.ySpeed, engine.yawSpeed, state == Walking ? 1 : 0);
}

Eigen::Vector4d Walk::getOrder() const
{
  // XXX: To update with new walk, what is the goal here?
  return Eigen::Vector4d(engine.xSpeed, engine.ySpeed, engine.yawSpeed, state == Walking ? 1 : 0);
}

void Walk::setRawOrder(double step, double lateral, double turn, bool enable)
{
  bind->node().setBool("walkEnable", enable);
  bind->node().setFloat("walkStep", step * 1000.0);
  bind->node().setFloat("walkLateral", lateral * 1000.0);
  bind->node().setFloat("walkTurn", rad2deg(turn));
}
void Walk::setRawOrder(const Eigen::Vector3d& params, bool enable)
{
  setRawOrder(params(0), params(1), params(2), enable);
}

rhoban_geometry::Point Walk::trunkToFlyingFoot(rhoban_geometry::Point point)
{
  rhoban::WalkEngine::FootPose flyingPose;
  double deltaY;

  if (engine.isLeftSupport)
  {
    flyingPose = engine.right.getPosition(timeSinceLastStep);
    deltaY = engine.right.trunkYOffset;
  }
  else
  {
    flyingPose = engine.left.getPosition(timeSinceLastStep);
    deltaY = engine.left.trunkYOffset;
  }

  rhoban_geometry::Point delta(0, -deltaY);
  delta.rotation(flyingPose.yaw);
  rhoban_geometry::Point trunkAfterStep(flyingPose.x + delta.x, flyingPose.y + delta.y);

  point.x -= trunkAfterStep.x;
  point.y -= trunkAfterStep.y;
  point = point.rotation(-flyingPose.yaw);

  return point;
}
