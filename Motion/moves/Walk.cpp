#include <math.h>
#include "Walk.h"
#include "Head.h"
#include "services/DecisionService.h"
#include "services/ModelService.h"
#include <scheduler/MoveScheduler.h>
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
  swingGainStart = 0.04;
  trunkPitch = 12;
  bootstrapSteps = 3;
  safeArmsRoll = 40;
  shouldBootstrap = false;
  armsEnabled = true;
  safeArmsRollEnabled = false;

  // Enables or disables the walk
  bind->bindNew("walkEnable", walkEnable, RhIO::Bind::PullOnly)->defaultValue(false);
  bind->bindNew("walkStep", walkStep, RhIO::Bind::PullOnly)->comment("Walk control Step [mm/step]")->defaultValue(0.0);
  bind->bindNew("walkLateral", walkLateral, RhIO::Bind::PullOnly)
      ->comment("Walk control Lateral [mm/step]")
      ->defaultValue(0.0);
  bind->bindNew("walkTurn", walkTurn, RhIO::Bind::PullOnly)->comment("Walk control Turn [deg/step]")->defaultValue(0.0);

  // Walk limits (to inform other moves about limits)
  bind->bindNew("maxRotation", maxRotation, RhIO::Bind::PullOnly)->defaultValue(maxRotation = 15);
  bind->bindNew("maxStep", maxStep, RhIO::Bind::PullOnly)->defaultValue(maxStep = 0.08);
  bind->bindNew("maxStepBackward", maxStepBackward, RhIO::Bind::PullOnly)->defaultValue(maxStepBackward = 0.04);
  bind->bindNew("maxLateral", maxLateral, RhIO::Bind::PullOnly)->defaultValue(maxLateral = 0.04);

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
  bind->bindNew("footYOffsetPerStepSizeY", engine.footYOffsetPerStepSizeY, RhIO::Bind::PullOnly)
      ->defaultValue(engine.footYOffsetPerStepSizeY);
  bind->bindNew("trunkPitch", trunkPitch, RhIO::Bind::PullOnly)->defaultValue(trunkPitch);

  // XXX: This feature can be deleted later if it is of no use
  bind->bindNew("bootstrapSteps", bootstrapSteps, RhIO::Bind::PullOnly)->defaultValue(bootstrapSteps);
  bind->bindNew("shouldBootstrap", shouldBootstrap, RhIO::Bind::PullOnly)->defaultValue(shouldBootstrap);

  // Acceleration limits
  bind->bindNew("maxDStepByCycle", maxDStepByCycle, RhIO::Bind::PullOnly)
      ->defaultValue(0.035)
      ->comment("Maximal difference between two steps [mm/step^2]");
  bind->bindNew("maxDLatByCycle", maxDLatByCycle, RhIO::Bind::PullOnly)
      ->defaultValue(0.03)
      ->comment("Maximal difference between two steps [mm/step^2]");
  bind->bindNew("maxDTurnByCycle", maxDTurnByCycle, RhIO::Bind::PullOnly)
      ->defaultValue(7)
      ->comment("Maximal difference between two steps [deg/step^2]");

  // Security parameters
  bind->bindNew("securityThreshold", securityThreshold, RhIO::Bind::PullOnly)
      ->defaultValue(0.065)
      ->minimum(0)
      ->maximum(1.0);
  bind->bindNew("securityPhase", securityPhase, RhIO::Bind::PullOnly)->defaultValue(0.05);

  // Kick parameters
  bind->bindNew("kickPending", kickPending, RhIO::Bind::PullOnly)->defaultValue(false);
  bind->bindNew("kickLeftPending", kickLeftPending, RhIO::Bind::PushAndPull)->defaultValue(false);
  bind->bindNew("kickRightPending", kickRightPending, RhIO::Bind::PushAndPull)->defaultValue(false);
  bind->bindNew("kickLeftFoot", kickLeftFoot, RhIO::Bind::PullOnly)->defaultValue(false);
  bind->bindNew("kickName", kickName, RhIO::Bind::PullOnly)->defaultValue("classic");
  bind->bindNew("kickCooldown", kickCooldown, RhIO::Bind::PullOnly)
      ->defaultValue(0.5)
      ->comment("Cooldown duration [s]");
  bind->bindNew("kickWarmup", kickWarmup, RhIO::Bind::PullOnly)->defaultValue(0.75)->comment("Warmup [s]");

  // Arms
  bind->bindNew("armsRoll", armsRoll, RhIO::Bind::PullOnly)->defaultValue(-5.0)->minimum(-20.0)->maximum(150.0);
  bind->bindNew("safeArmsRoll", safeArmsRoll, RhIO::Bind::PullOnly)
      ->defaultValue(safeArmsRoll)
      ->minimum(-20.0)
      ->maximum(150.0);
  bind->bindNew("elbowOffset", elbowOffset, RhIO::Bind::PullOnly)->defaultValue(-165.0)->minimum(-200.0)->maximum(30.0);
  bind->bindNew("armsEnabled", armsEnabled, RhIO::Bind::PullOnly)->defaultValue(armsEnabled);
  bind->bindNew("safeArmsRollEnabled", safeArmsRollEnabled, RhIO::Bind::PullOnly)->defaultValue(safeArmsRollEnabled);

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
  // Ensuring safety of head
  Move* head = getScheduler()->getMove("head");
  if (!head->isRunning())
  {
    walkLogger.log("Move 'head' is not running, starting it for safety");
    startMove("head", 0.5);
    Head* tmp = dynamic_cast<Head*>(head);
    if (tmp == nullptr)
    {
      throw std::logic_error("Failed to cast 'head' motion to 'Head' type");
    }
    tmp->setDisabled(true);
  }
  auto model = getServices()->model;
  engine.initByModel(model->model);

  bind->node().setBool("walkEnable", false);

  // Zeroing the orders
  bind->node().setFloat("walkStep", 0.0);
  bind->node().setFloat("walkLateral", 0.0);
  bind->node().setFloat("walkTurn", 0.0);

  // Cancelling eventual previous pending kicks
  bind->node().setBool("kickPending", false);
  bind->node().setBool("kickLeftPending", false);
  bind->node().setBool("kickRightPending", false);

  state = WalkNotWalking;
  kickState = KickNotKicking;
  bind->node().setBool("armsEnabled", true);
  smoothingArms = 0;
}

void Walk::onStop()
{
  getServices()->model->enableOdometry(false);
  state = WalkNotWalking;
  kickState = KickNotKicking;
}

void Walk::control(bool enable, double step, double lateral, double turn)
{
  if (isRunning())
  {
    step = bound(step, -maxStepBackward, maxStep);
    lateral = bound(lateral, -maxLateral, maxLateral);
    turn = bound(turn, -maxRotation, maxRotation);

    bind->node().setBool("walkEnable", enable);
    bind->node().setFloat("walkStep", step);
    bind->node().setFloat("walkLateral", lateral);
    bind->node().setFloat("walkTurn", turn);
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
  ModelService* modelService = getServices()->model;

  bind->pull();
  engine.trunkPitch = deg2rad(trunkPitch);

  stepKick(elapsed);

  modelService->enableOdometry(state != WalkNotWalking);

  if (state == WalkNotWalking)
  {
    // Walk is not enabled, just freezing the engine
    engine.riseGain = 0;
    engine.swingGain = 0;
    engine.stepSizeX = 0;
    engine.stepSizeY = 0;
    engine.stepSizeYaw = 0;
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
        if (state != WalkStarting)
        {
          timeSinceLastStep -= engine.stepDuration;
        }
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
                fabs(engine.stepSizeX) > 0.01 || fabs(engine.stepSizeY) > 0.01 || rad2deg(fabs(engine.stepSizeYaw)) > 3;
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
          engine.stepSizeX = 0;
          engine.stepSizeY = 0;
          engine.stepSizeYaw = 0;
        }
        else
        {
          // Updating engine speed according to acc. limits
          VariationBound::update(engine.stepSizeX, walkStep, maxDStepByCycle, 1);
          VariationBound::update(engine.stepSizeY, walkLateral, maxDLatByCycle, 1);
          VariationBound::update(engine.stepSizeYaw, deg2rad(walkTurn), deg2rad(maxDTurnByCycle), 1);
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

        if (Helpers::isFakeMode())
        {
          modelService->model.setSupportFoot(
              engine.isLeftSupport ? rhoban::HumanoidModel::Left : rhoban::HumanoidModel::Right, true);
        }
      }
    }
  }

  // Assigning to robot
  std::map<std::string, double> angles = engine.computeAngles(modelService->model, timeSinceLastStep);
  for (auto& entry : angles)
  {
    setAngle(entry.first, rad2deg(entry.second));
  }

  // Update arms
  stepArms(elapsed);

  bind->push();
}

void Walk::stepKick(float elapsed)
{
  if (kickLeftPending)
  {
    bind->node().setBool("kickLeftFoot", true);
    kickLeftFoot = true;
    kickPending = true;
    kickLeftPending = false;
  }

  if (kickRightPending)
  {
    bind->node().setBool("kickLeftFoot", false);
    kickLeftFoot = false;
    kickPending = true;
    kickRightPending = false;
  }

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
      // Forcing support foot in the model
      getServices()->model->model.setSupportFoot(
          kickLeftFoot ? rhoban::HumanoidModel::Right : rhoban::HumanoidModel::Left, true);

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
  bind->node().setBool("armsEnabled", enabled);
}

void Walk::enableSafeArmsRoll(bool enabled)
{
  bind->node().setBool("safeArmsRollEnabled", enabled);
}

void Walk::stepArms(double elapsed)
{
  smoothingArms = bound(smoothingArms + elapsed * (armsEnabled ? 3 : -3), 0, 1);

  // IMU Pitch to arms
  float imuPitch = rad2deg(getPitch());
  setAngle("left_shoulder_pitch", imuPitch * smoothingArms);
  setAngle("right_shoulder_pitch", imuPitch * smoothingArms);

  // Rolls to arms
  double roll;
  if (safeArmsRollEnabled)
  {
    roll = safeArmsRoll;
  }
  else
  {
    roll = armsRoll;
  }
  setAngle("left_shoulder_roll", roll * smoothingArms);
  setAngle("right_shoulder_roll", -roll * smoothingArms);

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
  bound << -maxStepBackward, -maxLateral, deg2rad(-maxRotation);
  return bound;
}

Eigen::Vector3d Walk::getMaxOrders() const
{
  Eigen::Vector3d bound;
  bound << maxStep, maxLateral, deg2rad(maxRotation);
  return bound;
}

Eigen::Vector3d Walk::getMinDeltaOrders() const
{
  Eigen::Vector3d bound;
  bound << -maxDStepByCycle, -maxDLatByCycle, deg2rad(-maxDTurnByCycle);
  return bound;
}

Eigen::Vector3d Walk::getMaxDeltaOrders() const
{
  Eigen::Vector3d bound;
  bound << maxDStepByCycle, maxDLatByCycle, deg2rad(maxDTurnByCycle);
  return bound;
}

Eigen::Vector4d Walk::getRawOrder() const
{
  return Eigen::Vector4d(engine.stepSizeX, engine.stepSizeY, engine.stepSizeYaw, state == Walking ? 1 : 0);
}

Eigen::Vector4d Walk::getOrder() const
{
  // XXX: To update with new walk, what is the goal here?
  return Eigen::Vector4d(engine.stepSizeX, engine.stepSizeY, engine.stepSizeYaw, state == Walking ? 1 : 0);
}

void Walk::setRawOrder(double step, double lateral, double turn, bool enable)
{
  bind->node().setBool("walkEnable", enable);
  bind->node().setFloat("walkStep", step);
  bind->node().setFloat("walkLateral", lateral);
  bind->node().setFloat("walkTurn", rad2deg(turn));
}
void Walk::setRawOrder(const Eigen::Vector3d& params, bool enable)
{
  setRawOrder(params(0), params(1), params(2), enable);
}

rhoban_geometry::Point Walk::trunkToFlyingFoot(rhoban_geometry::Point point)
{
  if (!isRunning())
  {
    return point;
  }

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
