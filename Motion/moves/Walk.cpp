#include <math.h>
#include "services/DecisionService.h"
#include "services/ModelService.h"
#include <scheduler/MoveScheduler.h>
#include "rhoban_utils/angle.h"
#include <rhoban_utils/logging/logger.h>
#include <rhoban_utils/control/variation_bound.h>
#include "Walk.h"
#include "Head.h"
#include "Arms.h"
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

Walk::Walk(Head* head, Arms* arms) : head(head), arms(arms), kick(nullptr)
{
  Move::initializeBinding();

  // Enables or disables the walk
  walkEnable = true;
  bind->bindNew("walkEnable", walkEnable, RhIO::Bind::PullOnly)->defaultValue(false);
  bind->bindNew("walkStep", walkStep, RhIO::Bind::PullOnly)->comment("Walk control Step [mm/step]")->defaultValue(0.0);
  bind->bindNew("walkLateral", walkLateral, RhIO::Bind::PullOnly)
      ->comment("Walk control Lateral [mm/step]")
      ->defaultValue(0.0);
  bind->bindNew("walkTurn", walkTurn, RhIO::Bind::PullOnly)->comment("Walk control Turn [deg/step]")->defaultValue(0.0);

  // Walk limits (to inform other moves about limits)
  bind->bindNew("maxRotation", maxRotation, RhIO::Bind::PullOnly)->defaultValue(15);
  bind->bindNew("maxStep", maxStep, RhIO::Bind::PullOnly)->defaultValue(0.08);
  bind->bindNew("maxStepBackward", maxStepBackward, RhIO::Bind::PullOnly)->defaultValue(0.04);
  bind->bindNew("maxLateral", maxLateral, RhIO::Bind::PullOnly)->defaultValue(0.04);

  // Walk engine parameters
  bind->bindNew("trunkXOffset", engine.trunkXOffset, RhIO::Bind::PullOnly)->defaultValue(engine.trunkXOffset);
  bind->bindNew("trunkZOffset", engine.trunkZOffset, RhIO::Bind::PullOnly)->defaultValue(engine.trunkZOffset);
  bind->bindNew("frequency", engine.frequency, RhIO::Bind::PullOnly)->defaultValue(engine.frequency);
  bind->bindNew("footYOffset", engine.footYOffset, RhIO::Bind::PullOnly)->defaultValue(engine.footYOffset);
  bind->bindNew("riseGain", engine.riseGain, RhIO::Bind::PullOnly)->defaultValue(engine.riseGain);
  bind->bindNew("riseDuration", engine.riseDuration, RhIO::Bind::PullOnly)->defaultValue(engine.riseDuration);
  bind->bindNew("swingGain", engine.swingGain, RhIO::Bind::PullOnly)->defaultValue(engine.swingGain);
  bind->bindNew("swingGainStart", swingGainStart, RhIO::Bind::PullOnly)->defaultValue(0.04);
  bind->bindNew("swingPhase", engine.swingPhase, RhIO::Bind::PullOnly)->defaultValue(engine.swingPhase);
  bind->bindNew("footYOffsetPerStepSizeY", engine.footYOffsetPerStepSizeY, RhIO::Bind::PullOnly)
      ->defaultValue(engine.footYOffsetPerStepSizeY);
  bind->bindNew("trunkPitch", trunkPitch, RhIO::Bind::PullOnly)->defaultValue(12);

  // XXX: This feature can be deleted later if it is of no use
  bind->bindNew("bootstrapSteps", bootstrapSteps, RhIO::Bind::PullOnly)->defaultValue(3);
  bind->bindNew("shouldBootstrap", shouldBootstrap, RhIO::Bind::PullOnly)->defaultValue(false);

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
  bind->bindNew("securityBlock", securityBlock, RhIO::Bind::PushOnly);

  timeSinceLastStep = 0;
}

void Walk::setKick(Kick* kick_)
{
  kick = kick_;
}

std::string Walk::getName()
{
  return "walk";
}

void Walk::onStart()
{
  bind->pull();

  // Ensuring safety of head
  if (!head->isRunning())
  {
    walkLogger.log("Move 'head' is not running, starting it for safety");
    startMove("head", 0.5);
    head->setDisabled(true);
  }
  startMove("arms", 0.5);

  arms->setArms(Arms::ArmsState::ArmsEnabled, true, true);

  auto model = getServices()->model;
  engine.initByModel(model->model);

  bind->node().setBool("walkEnable", false);

  // Zeroing the orders
  bind->node().setFloat("walkStep", 0.0);
  bind->node().setFloat("walkLateral", 0.0);
  bind->node().setFloat("walkTurn", 0.0);

  state = WalkNotWalking;
}

void Walk::onStop()
{
  Arms* arms = (Arms*)getMoves()->getMove("arms");
  arms->setArms(Arms::ArmsState::ArmsDisabled);
  getServices()->model->enableOdometry(false);
  state = WalkNotWalking;
}

void Walk::control(bool enable, double step, double lateral, double turn)
{
  if (isRunning())
  {
    if (kick != nullptr && kick->isRunning() && enable)
    {
      walkLogger.warning("Walk received order while kicking, ignoring");
      enable = false;
    }
    step = bound(step, -maxStepBackward, maxStep);
    lateral = bound(lateral, -maxLateral, maxLateral);
    turn = bound(turn, -maxRotation, maxRotation);

    bind->node().setBool("walkEnable", enable);
    bind->node().setFloat("walkStep", step);
    bind->node().setFloat("walkLateral", lateral);
    bind->node().setFloat("walkTurn", turn);
  }
}

bool Walk::isWalking()
{
  return state != WalkNotWalking;
}

void Walk::setShouldBootstrap(bool bootstrap)
{
  bind->node().setBool("shouldBootstrap", bootstrap);
}

void Walk::step(float elapsed)
{
  auto modelService = getServices()->model;

  bind->pull();

  engine.trunkPitch = deg2rad(trunkPitch);
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
    securityBlock = false;

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
          double maxD = maxDStepByCycle;
          if (fabs(walkStep) * 3 < fabs(engine.stepSizeX))
            maxD *= 3;
          VariationBound::update(engine.stepSizeX, walkStep, maxD, 1);

          maxD = maxDLatByCycle;
          if (fabs(walkLateral) * 3 < fabs(engine.stepSizeY))
            maxD *= 3;
          VariationBound::update(engine.stepSizeY, walkLateral, maxD, 1);

          maxD = maxDTurnByCycle;
          if (fabs(walkTurn) * 3 < fabs(engine.stepSizeYaw))
            maxD *= 3;
          VariationBound::update(engine.stepSizeYaw, deg2rad(walkTurn), deg2rad(maxD), 1);
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

  bind->push();
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

Eigen::Affine3d Walk::futureSelfToWorld()
{
  // Compute the foot frames in the trunk at the end of the current step
  auto supportFootToTrunk = engine.supportFoot().getPosition(engine.stepDuration).footToTrunk();
  auto flyingFootToTrunk = engine.flyingFoot().getPosition(engine.stepDuration).footToTrunk();

  // Compute the self to flying foot frame
  auto selfToFlyingFoot = Eigen::Affine3d::Identity();
  selfToFlyingFoot.translation().y() = -engine.flyingFoot().trunkYOffset;

  // Self to world frame
  auto supportToWorld = getServices()->model->model.supportToWorld;

  return supportToWorld * supportFootToTrunk.inverse() * flyingFootToTrunk * selfToFlyingFoot;
}
