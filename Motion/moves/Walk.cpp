#include <algorithm>
#include <math.h>
#include "Walk.h"
#include "Head.h"
#include <RhIO.hpp>
#include <rhoban_utils/logging/logger.h>
#include <rhoban_utils/angle.h>
#include <Model/HumanoidFloatingModel.hpp>
#include <rhoban_utils/control/variation_bound.h>
#include <services/LocalisationService.h>
#include <services/ModelService.h>
#include <services/DecisionService.h>
#include <Devices/PressureSensor.hpp>
#include <scheduler/MoveScheduler.h>
#include <rhoban_utils/control/variation_bound.h>
#include <rhoban_utils/nominal/nominal.h>
#include <cstring>
#include "Kick.h"

// #define MODE_NOMINAL

static rhoban_utils::Logger walkLogger("Walk");

using namespace rhoban_utils;

#ifdef MODE_NOMINAL
static Nominal nominal(4, 100);

static float phaseAverage(float p1, float w1, float p2, float w2)
{
    Angle a1(p1*360);
    Angle a2(p2*360);

    return Angle::weightedAverage(a1, w1, a2, w2).getValue()/360.0;
}

static float distancePhase(float p1, float p2)
{
    float d1 = fabs(p2-p1);
    float d2 = 1-d1;

    if (d1 < d2) return d1;
    else return d2;
}

static float changePhase(float p, float pTarget, float pDeltaMax)
{
    float d = distancePhase(p, pTarget);
    if (d < pDeltaMax) {
        return pTarget;
    } else {
        float pA = normalizePhase1(p+pDeltaMax);
        float pB = normalizePhase1(p-pDeltaMax);

        if (distancePhase(pTarget, pA) < distancePhase(pTarget, pB)) {
            return pA;
        } else {
            return pB;
        }
    }
}

static float normalizePhase1(float phase)
{
    while (phase >= 1.0) phase -= 1;
    while (phase < 0.0) phase += 1;
    return phase;
}
#endif

// Normalize phase in [-0.5, 0.5]
static float normalizePhase(float phase)
{
    while (phase >= 0.5) phase -= 1;
    while (phase <= -0.5) phase += 1;
    return phase;
}


static bool phasePassed(float before, float after, float phase)
{
    float diffBefore = normalizePhase(before-phase);
    float diffAfter = normalizePhase(after-phase);

    return (diffBefore <= 0) && (diffAfter >= 0);
}

Walk::Walk(Kick *kickMove)
    : kickMove(kickMove)
{
#ifdef USE_QUINTICWALK
    _params = _engine.getParameters();
#endif
    Move::initializeBinding();

#ifdef MODE_NOMINAL
    nominal.loadJson("nominal.json");
#endif

    RhIO::Root.newCommand("moves/walk/walkKick",
                          "request the walk to perform a kick",
                          [this](const std::vector<std::string> & args) -> std::string {
                            bool rightFoot = true;
                            std::string kickName = "classic";
                            if (args.size() >= 1 && args[0] == "left") {
                              rightFoot = false;
                            }
                            if (args.size() >= 2) {
                              kickName = args[1];
                            }
                            kick(rightFoot, kickName);
                            return "Performing a kick";
                          });

#ifndef USE_QUINTICWALK
    bind->bindNew("freq", params.freq, RhIO::Bind::PullOnly)
        ->comment("Walk frequency")->persisted(true)
        ->defaultValue(2)->minimum(0.1)->maximum(10.0)
        ;
    bind->bindNew("supportPhaseRatio", params.supportPhaseRatio, RhIO::Bind::PullOnly)
        ->comment("Support phase ratio")->persisted(true)
        ->defaultValue(0.0)->minimum(0.0)->maximum(1.0);
#else
    bind->bindNew("freq", _params("freq"), RhIO::Bind::PullOnly)
        ->comment("Walk frequency")->persisted(true)
        ->defaultValue(2)->minimum(0.1)->maximum(10.0)
        ;
    bind->bindNew("supportPhaseRatio", _params("doubleSupportRatio"), RhIO::Bind::PullOnly)
        ->comment("Support phase ratio")->persisted(true)
        ->defaultValue(0.0)->minimum(0.0)->maximum(1.0);
#endif

    // Offsets
#ifndef USE_QUINTICWALK
    bind->bindNew("footYOffset", params.footYOffset, RhIO::Bind::PullOnly)
        ->comment("Front Y offset")->persisted(true)
        ->defaultValue(0.01)->minimum(-0.1)->maximum(0.1)
        ;
#else
    bind->bindNew("footYOffset", _footYOffset, RhIO::Bind::PullOnly)
        ->comment("Front Y offset")->persisted(true)
        ->defaultValue(0.01)->minimum(-0.1)->maximum(0.1)
        ;
#endif
    bind->bindNew("trunkYOffset", trunkYOffset, RhIO::Bind::PullOnly)
        ->comment("Trunk Y offset")->persisted(true)
        ->defaultValue(0.00)->minimum(-0.1)->maximum(0.1)
        ;
#ifndef USE_QUINTICWALK
    bind->bindNew("trunkZOffset", params.trunkZOffset, RhIO::Bind::PullOnly)
        ->comment("Trunk Z offset")->persisted(true)
        ->defaultValue(0.02)->minimum(0.0)->maximum(0.1)
        ;
    bind->bindNew("trunkRoll", trunkRoll, RhIO::Bind::PullOnly)
        ->comment("Roll of the trunk")->persisted(true)
        ->defaultValue(0.0)->minimum(-45.0)->maximum(45.0)
        ;
#else
    bind->bindNew("trunkZOffset", _trunkZOffset, RhIO::Bind::PullOnly)
        ->comment("Trunk Z offset")->persisted(true)
        ->defaultValue(0.02)->minimum(0.0)->maximum(0.1)
        ;
#endif

    // Splines
#ifndef USE_QUINTICWALK
    bind->bindNew("stepUpVel", params.stepUpVel, RhIO::Bind::PullOnly)
        ->comment("Step up velocity")->persisted(true)
        ->defaultValue(2.0)->minimum(-5.0)->maximum(5.0)
        ;
    bind->bindNew("stepDownVel", params.stepDownVel, RhIO::Bind::PullOnly)
        ->comment("Step down velocity")->persisted(true)
        ->defaultValue(2.0)->minimum(-5.0)->maximum(5.0)
        ;
    bind->bindNew("riseUpVel", params.riseUpVel, RhIO::Bind::PullOnly)
        ->comment("Rise up velocity")->persisted(true)
        ->defaultValue(2.0)->minimum(-5.0)->maximum(5.0)
        ;
    bind->bindNew("riseDownVel", params.riseDownVel, RhIO::Bind::PullOnly)
        ->comment("Rise down velocity")->persisted(true)
        ->defaultValue(2.0)->minimum(-5.0)->maximum(5.0)
        ;
    bind->bindNew("riseGain", params.riseGain, RhIO::Bind::PullOnly)
        ->comment("Rise gain")->persisted(true)
        ->defaultValue(0.02)->minimum(0.0)->maximum(0.1)
        ;
#else
    bind->bindNew("footApexPhase", _params("footApexPhase"), RhIO::Bind::PullOnly)
        ->persisted(true)
        ->defaultValue(_params("footApexPhase"))->minimum(0.0)->maximum(1.0)
        ;
    bind->bindNew("footOvershootRatio", _params("footOvershootRatio"), RhIO::Bind::PullOnly)
        ->persisted(true)
        ->defaultValue(_params("footOvershootRatio"))->minimum(0.0)->maximum(1.0)
        ;
    bind->bindNew("footOvershootPhase", _params("footOvershootPhase"), RhIO::Bind::PullOnly)
        ->persisted(true)
        ->defaultValue(_params("footOvershootPhase"))->minimum(0.0)->maximum(1.0)
        ;
    bind->bindNew("riseGain", _params("footRise"), RhIO::Bind::PullOnly)
        ->comment("Rise gain")->persisted(true)
        ->defaultValue(0.02)->minimum(0.0)->maximum(0.1)
        ;
#endif
    
    bind->bindNew("startPhase", startPhase, RhIO::Bind::PullOnly)
        ->comment("Start phase")->persisted(true)
        ->defaultValue(0.0)->minimum(0.0)->maximum(1.0)
        ;

    // Swing
    bind->bindNew("swingGain", swingGain, RhIO::Bind::PullOnly)
        ->comment("Swing gain")->persisted(true)
        ->defaultValue(0.0)->minimum(-0.1)->maximum(0.1)
        ;
#ifndef USE_QUINTICWALK
    bind->bindNew("swingRollGain", swingRollGain, RhIO::Bind::PullOnly)
        ->comment("Swing roll gain")->persisted(true)
        ->defaultValue(0.0)->minimum(-45)->maximum(45)
        ;
#endif
    bind->bindNew("swingPhase", swingPhase, RhIO::Bind::PullOnly)
        ->comment("Swing phase")->persisted(true)
        ->defaultValue(0.0)->minimum(0.0)->maximum(1.0)
        ;
#ifndef USE_QUINTICWALK
    bind->bindNew("swingPause", params.swingPause, RhIO::Bind::PullOnly)
        ->comment("Swing pause")->persisted(true)
        ->defaultValue(0.0)->minimum(0.0)->maximum(0.5)
        ;
    bind->bindNew("swingVel", params.swingVel, RhIO::Bind::PullOnly)
        ->comment("Swing velocity")->persisted(true)
        ->defaultValue(1.0)->minimum(0.0)->maximum(5.0)
        ;
#else
    bind->bindNew("swingPause", _params("trunkPause"), RhIO::Bind::PullOnly)
        ->comment("Swing pause")->persisted(true)
        ->defaultValue(0.0)->minimum(0.0)->maximum(0.5)
        ;
#endif

    // Walk phase
    bind->bindNew("phase", phase, RhIO::Bind::PushOnly);
    // Tresholds
    bind->bindNew("securityTreshold", securityTreshold)
        ->defaultValue(0.065)->minimum(0)->maximum(1.0)
        ->persisted(true)
        ;
    bind->bindNew("securityPhase", securityPhase)
        ->defaultValue(0.1)
        ->persisted(true)
        ;
    // Foot roll for compensation
    bind->bindNew("compensateRoll", compensateRoll)
        ->defaultValue(0.0)->minimum(-45)->maximum(45)
        ->persisted(true)
        ;
    leftRoll = 0;
    rightRoll = 0;

    // Shooting
    bind->bindNew("shootPhase", shootPhase, RhIO::Bind::PullOnly)
            ->comment("Shooting step")->defaultValue(0)
            ->persisted(true)
            ;
    bind->bindNew("shootAfterPhase", shootAfterPhase, RhIO::Bind::PullOnly)
            ->comment("Shooting step")->defaultValue(0.25)
            ->persisted(true)
            ;

    // Arms
    bind->bindNew("armsRoll", armsRoll, RhIO::Bind::PullOnly)
        ->defaultValue(18.0)->minimum(-20.0)->maximum(150.0)
        ->persisted(true)
        ;
    bind->bindNew("armsPitch", armsPitch, RhIO::Bind::PullOnly)
        ->defaultValue(0.0)->minimum(0.0)->maximum(150.0)
        ->persisted(true)
        ;
    bind->bindNew("elbowOffset", elbowOffset, RhIO::Bind::PullOnly)
        ->defaultValue(-150.0)->minimum(-200.0)->maximum(30.0)
        ->persisted(true)
        ;
    

    // Trimming
    bind->bindNew("stepTrim", stepTrim, RhIO::Bind::PullOnly)
        ->defaultValue(0.0)->persisted(true)
        ;
    bind->bindNew("lateralTrim", lateralTrim, RhIO::Bind::PullOnly)
        ->defaultValue(0.0)->persisted(true)
        ;
    bind->bindNew("turnTrim", turnTrim, RhIO::Bind::PullOnly)
        ->defaultValue(0.0)->persisted(true)
        ;
    bind->bindNew("smoothingStep", smoothingStep, RhIO::Bind::PushOnly)
        ;
    bind->bindNew("smoothingLateral", smoothingLateral, RhIO::Bind::PushOnly)
        ;
    bind->bindNew("smoothingTurn", smoothingTurn, RhIO::Bind::PushOnly)
        ;

    bind->bindNew("trunkXOffset_forward", trunkXOffset_forward, RhIO::Bind::PullOnly)
        ->comment("Trunk X offset")->persisted(true)
        ->defaultValue(0.02)->minimum(-0.1)->maximum(0.1)
        ;
    bind->bindNew("trunkXOffset_backward", trunkXOffset_backward, RhIO::Bind::PullOnly)
        ->comment("Trunk X offset")->persisted(true)
        ->defaultValue(0.01)->minimum(-0.1)->maximum(0.1)
        ;

    bind->bindNew("trunkPitch_forward", trunkPitch_forward, RhIO::Bind::PullOnly)
        ->comment("Trunk Pitch")->persisted(true)
        ->defaultValue(0.00)->minimum(-30)->maximum(30)
        ;
    bind->bindNew("trunkPitch_backward", trunkPitch_backward, RhIO::Bind::PullOnly)
        ->comment("Trunk Pitch")->persisted(true)
        ->defaultValue(0.00)->minimum(-30)->maximum(30)
        ;


    bind->bindNew("P_stepXOffset", P_stepXOffset, RhIO::Bind::PullOnly)
        ->comment("P factor on trunkXOffset related to step")->persisted(true)
        ->defaultValue(0);


    bind->bindNew("P_stepPitch", P_stepPitch, RhIO::Bind::PullOnly)
        ->comment("P factor on trunkPitch related to step")->persisted(true)
        ->defaultValue(0);



    bind->bindNew("P_stepXOffset_back", P_stepXOffset_back, RhIO::Bind::PullOnly)
        ->comment("P factor on trunkXOffset related to step (backward)")->persisted(true)
        ->defaultValue(0);


    bind->bindNew("P_stepPitch_back", P_stepPitch_back, RhIO::Bind::PullOnly)
        ->comment("P factor on trunkPitch related to step (backward)")->persisted(true)
        ->defaultValue(0);


    bind->bindNew("smoothTransition", smoothTransition, RhIO::Bind::PullOnly)
        ->comment("Smoothing factor when changing parameters")->persisted(true)
        ->defaultValue(0.5)->minimum(0)->maximum(1)
        ;

    bind->bindNew("smoothCommands", smoothCommands, RhIO::Bind::PullOnly)
            ->comment("Maximum step acceleration")->persisted(true)
            ->defaultValue(0.9)->minimum(0)->maximum(1)
            ;

    // Walk control
    bind->bindNew("walkEnable", walkEnable, RhIO::Bind::PullOnly)
            ->comment("Walk control Enable")->defaultValue(false);
    bind->bindNew("walkStep", walkStep, RhIO::Bind::PullOnly)
            ->comment("Walk control Step [mm/step]")->defaultValue(0.0);
    bind->bindNew("walkLateral", walkLateral, RhIO::Bind::PullOnly)
            ->comment("Walk control Lateral [mm/step]")->defaultValue(0.0);
    bind->bindNew("walkTurn", walkTurn, RhIO::Bind::PullOnly)
            ->comment("Walk control Turn [deg/step]")->defaultValue(0.0);

    // Kicks
    bind->bindNew("walkKickLeft", walkKickLeft, RhIO::Bind::PullOnly)
            ->comment("Kick left")->defaultValue(false);
    bind->bindNew("walkKickRight", walkKickRight, RhIO::Bind::PullOnly)
            ->comment("Kick right")->defaultValue(false);
    bind->bindNew("walkKickName", walkKickName, RhIO::Bind::PullOnly)
            ->comment("Kick name")->defaultValue("classic");

    // Shoot warmup and cool down
    bind->bindNew("cooldown", cooldown, RhIO::Bind::PullOnly)
        ->defaultValue(1)->comment("Cooldown duration [s]")->persisted(true);
    bind->bindNew("warmup", warmup, RhIO::Bind::PullOnly)
        ->defaultValue(1)->comment("Warmup [s]")->persisted(true);
        
#ifndef USE_QUINTICWALK
    bind->bindNew("paramsStepGain", params.stepGain, RhIO::Bind::PushOnly)
        ->comment("IKWalk really used step parameters");
    bind->bindNew("paramsLateralGain", params.lateralGain, RhIO::Bind::PushOnly)
        ->comment("IKWalk really used lateral parameters");
    bind->bindNew("paramsTurnGain", params.turnGain, RhIO::Bind::PushOnly)
        ->comment("IKWalk really used turn parameters");
#else
    bind->bindNew("paramsStepGain", _orders.x(), RhIO::Bind::PushOnly)
        ->comment("IKWalk really used step parameters");
    bind->bindNew("paramsLateralGain", _orders.y(), RhIO::Bind::PushOnly)
        ->comment("IKWalk really used lateral parameters");
    bind->bindNew("paramsTurnGain", _orders.z(), RhIO::Bind::PushOnly)
        ->comment("IKWalk really used turn parameters");
#endif

    bind->node().newFloat("ratio");
    
    // Speed limits
    bind->bindNew("maxRotation", maxRotation, RhIO::Bind::PullOnly)
        ->defaultValue(20.0);

    bind->bindNew("maxStep [mm/step]", maxStep, RhIO::Bind::PullOnly)
        ->defaultValue(45.0);

    bind->bindNew("maxStepBackward [mm/step]", maxStepBackward, RhIO::Bind::PullOnly)
        ->defaultValue(20.0);

    bind->bindNew("maxLateral [mm/step]", maxLateral, RhIO::Bind::PullOnly)
        ->defaultValue(30.0);

    bind->bindNew("maxDStepByCycle", maxDStepByCycle, RhIO::Bind::PullOnly)
      ->defaultValue(10)
      ->comment("Maximal difference between two steps [mm/step^2]");
    bind->bindNew("maxDLatByCycle", maxDLatByCycle, RhIO::Bind::PullOnly)
        ->defaultValue(5)
        ->comment("Maximal difference between two steps [mm/step^2]");
    bind->bindNew("maxDTurnByCycle", maxDTurnByCycle, RhIO::Bind::PullOnly)
        ->defaultValue(3)
        ->comment("Maximal difference between two steps [deg/step^2]");

    // Dont walk flag
    bind->bindNew("dontWalk", dontWalk, RhIO::Bind::PullOnly)
        ->defaultValue(false);


#ifdef MODE_NOMINAL
    bind->node().newFloat("guessPhase");
    bind->node().newFloat("nominalScore");
    bind->node().newFloat("phaseMaxV")
        ->defaultValue(0.1);
    bind->node().newBool("nominalError");
#endif

    // Zeroing extras
#ifndef USE_QUINTICWALK
    params.extraLeftX = 0;
    params.extraLeftY = 0;
    params.extraLeftZ = 0;
    params.extraLeftYaw = 0;
    params.extraLeftPitch = 0;
    params.extraLeftRoll = 0;
    params.extraRightX = 0;
    params.extraRightY = 0;
    params.extraRightZ = 0;
    params.extraRightYaw = 0;
    params.extraRightPitch = 0;
    params.extraRightRoll = 0;
#endif

    bind->pull();
}
        
double Walk::getPhase() const
{
    return phase;
}
       
double Walk::getLastPhase() const
{
    return lastPhase;
}

bool Walk::isNewStep() const
{
  return phasePassed(lastPhase, phase, 0.0)
    || phasePassed(lastPhase, phase, 0.5);
}
        
Eigen::Vector4d Walk::getRawOrder() const
{
#ifndef USE_QUINTICWALK
    return Eigen::Vector4d(
        params.stepGain, 
        params.lateralGain, 
        params.turnGain,
        params.enabledGain);
#else
    return Eigen::Vector4d(
        _orders.x(), 
        _orders.y(), 
        _orders.z(),
        _isEnabled);
#endif
}

Eigen::Vector4d Walk::getOrder() const
{
    return Eigen::Vector4d(
        smoothingStep / 1000, 
        smoothingLateral / 1000, 
        deg2rad(smoothingTurn),
        smoothing);
}
        
void Walk::setRawOrder(
    double step, double lateral, 
    double turn, bool enable)
{
    bind->node().setBool("walkEnable", enable);
    bind->node().setFloat("walkStep", step*1000.0);
    bind->node().setFloat("walkLateral", lateral*1000.0);
    bind->node().setFloat("walkTurn", rad2deg(turn));
}       
void Walk::setRawOrder(const Eigen::Vector3d & params, bool enable)
{
  setRawOrder(params(0), params(1), params(2), enable);
}

Eigen::Vector3d Walk::getMinOrders() const
{
  Eigen::Vector3d bound;
  bound << -maxStepBackward / 1000, -maxLateral / 1000, deg2rad(-maxRotation);
  return bound ;
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

std::string Walk::getName()
{
    return "walk";
}

static double bound(double a, double min, double max)
{
    if (a < min) return min;
    if (a > max) return max;

    return a;
}

void Walk::control(bool enable, double step, double lateral, double turn)
{
    if (isRunning()) {
        if (dontWalk) {
            enable = false;
        }

        bind->node().setBool("walkEnable", enable);
        bind->node().setFloat("walkStep", bound(step, -maxStepBackward, maxStep));
        bind->node().setFloat("walkLateral", bound(lateral, -maxLateral, maxLateral));
        bind->node().setFloat("walkTurn", bound(turn, -maxRotation, maxRotation));
    }
}

void Walk::kick(bool rightFoot, const std::string & kickName)
{
    walkLogger.log("Kick requested");
    if (rightFoot) {
        shouldKickRight = true;
    } else {
        shouldKickLeft = true;
    }

    bind->node().setStr("walkKickName", kickName);
}

bool Walk::isKicking()
{
    return shouldKickLeft || shouldKickRight || shootingLeft || shootingRight;
}

void Walk::onStart()
{
#ifdef USE_QUINTICWALK
    //Compute trunk height on all DOF set to zero
    Eigen::VectorXd cpyDOF = 
        Helpers::getServices()->model->goalModel()
        .get().getDOFVect();
    Helpers::getServices()->model->goalModel()
        .get().setDOFZeros();
    _trunkHeight = 
        Helpers::getServices()->model->goalModel()
        .get().position("trunk", "left_foot_tip").z();
    _footDistance = 
        Helpers::getServices()->model->goalModel()
        .get().position("left_foot_tip", "right_foot_tip").y();
    Helpers::getServices()->model->goalModel().get().setDOFVect(cpyDOF);
#endif

    // Ensuring safety of head
    Move * head  = getScheduler()->getMove("head");
    if (!head->isRunning()) {
        walkLogger.log("Move 'head' is not running, starting it for safety");
        startMove("head", 0.5);
        Head * tmp = dynamic_cast<Head *>(head);
        if (tmp == nullptr) {
          throw std::logic_error("Failed to cast 'head' motion to 'Head' type");
        }
        tmp->setDisabled(true);
    }
    // Default values for variables
    bind->pull();
    lastWalkEnable = false;
    walkEnable = false;
    walkStep = 0.0;
    walkLateral = 0.0;
    walkTurn = 0.0;
    bind->node().setBool("walkKickLeft", false);
    bind->node().setBool("walkKickRight", false);
    walkKickLeft = false;
    walkKickRight = false;
    walkKickName = "classic";
    smoothingTurn = smoothingStep = smoothingLateral = 0;
    smoothing = 0;
    shouldKickLeft = false;
    shouldKickRight = false;
    prevKickLeft = false;
    prevKickRight = false;
    shootingLeft = false;
    shootingRight = false;
    isWarmingUp = false;
    isCoolingDown = false;
    waitT = 0;
    phase = 0;
    lastPhase = 0;

#ifndef USE_QUINTICWALK
    params.swingRollGain     = deg2rad(swingRollGain);
    params.swingGain         = swingGain;
    params.swingPhase        = swingPhase;
    trunkRoll_pose           = deg2rad(trunkRoll);
    params.trunkPitch        = 0.0;
    params.trunkXOffset      = 0.0;
    params.trunkYOffset      = trunkYOffset;
#else
    _params("footDistance") = 2.0*_footYOffset + _footDistance;
    _params("trunkSwing")        = swingGain/std::fabs(_params("footDistance"));
    _params("trunkPhase")        = swingPhase;
    _params("trunkPitch")        = 0.0;
    _params("trunkXOffset")      = 0.0;
    _params("trunkYOffset")      = trunkYOffset;
    _params("trunkHeight")       = _trunkHeight - _trunkZOffset;
    _securityEnabled = false;
    _engine.setParameters(_params);
    _engine.setOrders(Eigen::Vector3d(0.0, 0.0, 0.0), false);
    _engine.forceRebuildTrajectories();
#endif

    //security for arms. TODO should be a init command.
    setTorqueLimit("left_shoulder_pitch", 0.2);
    setTorqueLimit("right_shoulder_pitch", 0.2);
    setTorqueLimit("left_shoulder_roll", 0.2);
    setTorqueLimit("right_shoulder_roll", 0.2);

    //Turn off the control on rhio variables
    control(false);
    bind->push();
}

void Walk::onStop()
{
    Helpers::getServices()->model->setReadBaseUpdate(false);
}

void Walk::updateParams(double factor, float step, float lateral, float turn)
{
#ifndef USE_QUINTICWALK
    float targetSwingRollGain = deg2rad(swingRollGain);
    float targetTrunkRoll = deg2rad(trunkRoll);
#endif
    float targetSwingGain = swingGain;
    float targetSwingPhase = swingPhase;
    float targetTrunkPitch;
    float targetTrunkXOffset;
    float targetTrunkYOffset = trunkYOffset;

    if (step > 0.1) {
        targetTrunkXOffset = trunkXOffset_forward + step*P_stepXOffset;
        targetTrunkPitch = deg2rad(trunkPitch_forward) + deg2rad(step*P_stepPitch);
    } else {
        targetTrunkXOffset = trunkXOffset_backward + step*P_stepXOffset_back;
        targetTrunkPitch = deg2rad(trunkPitch_backward) + deg2rad(step*P_stepPitch_back);
    }

#ifndef USE_QUINTICWALK
    params.swingRollGain     = factor*params.swingRollGain     + (1-factor)*targetSwingRollGain;
    params.swingGain         = factor*params.swingGain         + (1-factor)*targetSwingGain;
    params.swingPhase        = factor*params.swingPhase        + (1-factor)*targetSwingPhase;
    trunkRoll_pose           = factor*trunkRoll_pose           + (1-factor)*targetTrunkRoll;
    params.trunkPitch        = factor*params.trunkPitch        + (1-factor)*targetTrunkPitch;
    params.trunkXOffset      = factor*params.trunkXOffset      + (1-factor)*targetTrunkXOffset;
    params.trunkYOffset      = factor*params.trunkYOffset      + (1-factor)*targetTrunkYOffset;
#else
    _params("footDistance") = 2.0*_footYOffset + _footDistance;
    _params("trunkSwing")     = 
        (factor*_params("trunkSwing")*std::fabs(_params("footDistance")) + (1-factor)*targetSwingGain)/std::fabs(_params("footDistance"));
    _params("trunkPhase")     = factor*_params("trunkPhase")        + (1-factor)*targetSwingPhase;
    _params("trunkPitch")     = factor*_params("trunkPitch")        + (1-factor)*targetTrunkPitch;
    _params("trunkXOffset")   = factor*_params("trunkXOffset")      + (1-factor)*targetTrunkXOffset;
    _params("trunkYOffset")   = factor*_params("trunkYOffset")      + (1-factor)*targetTrunkYOffset;
    _params("trunkHeight")    = _trunkHeight - _trunkZOffset;
#endif
}

void Walk::step(float elapsed)
{
    auto &decision = getServices()->decision;
    auto &model = getServices()->model;

#ifndef USE_QUINTICWALK
    params.extraLeftX = 0;
    params.extraLeftY = 0;
    params.extraLeftZ = 0;
    params.extraRightX = 0;
    params.extraRightY = 0;
    params.extraRightZ = 0;
#endif

    bind->pull();

    // Getting orders
    float lateral= walkLateral;
    float step = walkStep;
    float turn = walkTurn;

    // Not sending order until we started walking
    bool forbidOrders = false;
    if (isFading() || smoothing < 0.95 || shootingLeft || shootingRight) {
        forbidOrders = true;
        lateral = step = turn = 0;
        smoothingTurn = smoothingStep = smoothingLateral = 0;
    }

    bool walkActuallyEnable = (smoothing > 0.05);
    // Forcing to start at specific phase
    if (walkActuallyEnable && !lastWalkEnable) {
        phase = startPhase;
        lastPhase = phase;
    }
    lastWalkEnable = walkActuallyEnable;

    //TODO: LH: lateral and turn are not used in this function
    updateParams(smoothTransition, smoothingStep, smoothingLateral, smoothingTurn);

    // Smoothing
    smoothing = smoothing*smoothCommands + (walkEnable ? 1 : 0)*(1.0-smoothCommands);
    bool newStep = isNewStep();
    if (!forbidOrders && newStep) {
        VariationBound::update(smoothingStep, step, maxDStepByCycle, 1);
        VariationBound::update(smoothingLateral, lateral, maxDLatByCycle, 1);
        VariationBound::update(smoothingTurn, turn, maxDTurnByCycle, 1);
    }
    bool isEnabled = (smoothing > 0.95);

    // Setting real parameters used gains
#ifndef USE_QUINTICWALK
    params.enabledGain = smoothing;
    if (forbidOrders) {
      params.stepGain = 0;
      params.lateralGain = 0;
      params.turnGain = 0;
    }
    else if (newStep) {
      params.stepGain = (stepTrim+smoothingStep)/1000.0;
      params.lateralGain = (lateralTrim+smoothingLateral)/1000.0;
      params.turnGain = deg2rad(turnTrim+smoothingTurn);
    }
#else
    _isEnabled = (smoothing > 0.5);
    if (forbidOrders) {
        _orders.setZero();
    }
    else if (newStep) {
        _orders.x() = (stepTrim+smoothingStep)/1000.0;
        _orders.y() = (lateralTrim+smoothingLateral)/1000.0;
        _orders.z() = deg2rad(turnTrim+smoothingTurn);
    }
#endif
    lastPhase = phase;

    // Detecting 0->1 to trigger kicking on the lefr or right
    if (walkKickLeft && !prevKickLeft) {
        shouldKickLeft = true;
    }
    prevKickLeft = walkKickLeft;
    if (walkKickRight && !prevKickRight) {
        shouldKickRight = true;
    }
    prevKickRight = walkKickRight;

#ifndef USE_QUINTICWALK
    // Update the robot base only if walk is enabled
    model->setReadBaseUpdate(smoothing > 0.1);
    params.trunkRoll = trunkRoll_pose;
#else
    model->setReadBaseUpdate(_engine.isEnabled());
#endif
    
#ifndef USE_QUINTICWALK
    waitT += elapsed;
#else
    if (!_engine.isEnabled()) {
        waitT += elapsed;
    }
#endif

    if (shootingLeft || shootingRight) {
        if (isWarmingUp) {
            if (waitT > warmup && !decision->freezeKick) {
                startMove("kick", 0.0);
                isWarmingUp = false;
                shootT = 0;
            }
        } else if (isCoolingDown) {
            if (waitT > cooldown) {
                isCoolingDown = false;
                endShoot();
                if (shootingLeft) {
                    phase = 0.5+shootAfterPhase;
                } else {
                    phase = shootAfterPhase;
                }
            }
        } else if (kickMove->over) {
            isCoolingDown = true;
            waitT = 0;
            stopMove("kick", 0.0);
        }

#ifndef USE_QUINTICWALK
        params.enabledGain = smoothing = 0;
        params.stepGain = 0;
        params.riseGain = 0;
        params.swingGain = 0;
#else
        _isEnabled = false;
        _orders.setZero();
#endif
    }
    
    auto& pressureLeft = getScheduler()->getManager()->dev<RhAL::PressureSensor4>("left_pressure");
    auto& pressureRight = getScheduler()->getManager()->dev<RhAL::PressureSensor4>("right_pressure");
    double total = pressureLeft.getWeight() + pressureRight.getWeight();
    double y = 0;
    if (total > 0) {
        y = 
            (
            pressureLeft.getWeight()*(pressureLeft.getY() + 0.07) +
            pressureRight.getWeight()*(pressureRight.getY() - 0.07)
            )/total
            ;
    }
    bind->node().setFloat("ratio", y);

#ifdef MODE_NOMINAL
    double nominalScore = 0;
    double guessPhase = 0;
    {
        auto& imu = getScheduler()->getManager()->dev<RhAL::GY85>("imu");
        Eigen::VectorXd v(8);
        v << 
        y, 
          imu.getRoll(), 
          imu.getGyroX(), 
          imu.getAccY()/*,
          this->Helpers::getError("right_ankle_pitch"), this->Helpers::getError("right_ankle_roll"),
          this->Helpers::getError("left_ankle_roll"), this->Helpers::getError("left_ankle_pitch")
          */
          ;
        nominalScore = nominal.getBin(phase).score(v);
        guessPhase = nominal.guessT(v);
        bind->node().setFloat("guessPhase", guessPhase);
        bind->node().setFloat("nominalScore", nominalScore);
    }
    bind->node().setBool("nominalError", nominalScore > securityTreshold);
#endif

    float prevPhase = phase;
#ifdef USE_QUINTICWALK
    _engine.setParameters(_params);
    _engine.setOrders(_orders, _isEnabled);
    if (!_securityEnabled) {
        _engine.update(elapsed);
    }
    phase = _engine.getPhase();
    bool isSuccess = _engine.assignModel(model->goalModel());
#else
    bool isSuccess = Leph::IKWalk::walk(
        model->goalModel().get(), params, phase, elapsed);
#endif
    if (isSuccess) {
        bool securityBlock = false;

        // If walk is enabled and robot is not shooting: check security
        if (isEnabled && !shootingRight && !shootingLeft) {
#ifdef MODE_NOMINAL
            if (nominalScore > securityTreshold) {
                securityBlock = true;
            }
#else
            // Right foot security
            if (phasePassed(prevPhase, phase, securityPhase)) {
                if (y > securityTreshold) {
                    securityBlock = true;
                }
            }
            // Left foot security
            double p = 0.5+securityPhase;
            if (p > 1) p -= 1;
            if (phasePassed(prevPhase, phase, p)) {
                if (y < -securityTreshold) {
                    securityBlock = true;
                }
            }
#endif
        }

        if (!securityBlock && !shootingRight && !shootingLeft) {
            // Right kick phase
            if (!isEnabled || phasePassed(prevPhase, phase, shootPhase)) {
                if (shouldKickRight) {
                    updateShoot(false, walkKickName);
                    isWarmingUp = true;
                    shootT = 0;
                    waitT = 0;
                    shootingRight = true;
                    shouldKickRight = false;
                }
            }

            // Left kick phase
            if (!isEnabled || phasePassed(prevPhase, phase, 0.5+shootPhase)) {
                if (shouldKickLeft) {
                    updateShoot(true, walkKickName);
                    isWarmingUp = true;
                    shootT = 0;
                    waitT = 0;
                    shootingLeft = true;
                    shouldKickLeft = false;
                }
            }
        }

#ifndef USE_QUINTICWALK
        // Arms
        float targetLeftRoll = 0;
        float targetRightRoll = 0;
        leftRoll = leftRoll*0.95 + targetLeftRoll*0.05;
        params.extraLeftRoll = deg2rad(leftRoll);
        rightRoll = rightRoll*0.95 + targetRightRoll*0.05;
        params.extraRightRoll = deg2rad(rightRoll);
#else
        _securityEnabled = securityBlock;
#endif

        //if (!kickMove->isRunning() || kickMove->over) {
            model->flushLegs(_smoothing);
        //}

        if (securityBlock) {
#ifdef MODE_NOMINAL
            //float v = (phase-prevPhase);
            double phaseMaxV = bind->node().getFloat("phaseMaxV")*elapsed;
            //double op = phase;
            //printf("prev=%g, phase=%g, guessPhase=%g, nv=%g, pmv=%g, ", prevPhase, phase, guessPhase, v, phaseMaxV);
            phase = changePhase(phase, guessPhase, phaseMaxV);
            //printf("result=%g, RV=%g\n", phase, phase-op);
#else
            phase = prevPhase;
#endif
        }
    }

    auto &sim = model->readModel();
    auto &simModel = sim.get();

    // Pitch to arms
    float dPitch = rad2deg(getPitch());
    setAngle("left_shoulder_pitch", armsPitch+dPitch);
    setAngle("right_shoulder_pitch", armsPitch+dPitch);
    // Rolls to arms
    setAngle("left_shoulder_roll", armsRoll);
    setAngle("right_shoulder_roll", -armsRoll);
    // Elbows
    setAngle("left_elbow", elbowOffset);
    setAngle("right_elbow", elbowOffset);

    //Do logging
    _logs.logServos();
    _logs.logSensors();
    _logs.logPressure();
    _logs.logBase(simModel);
    _logs.logModel(sim);
    _logs.logData("time:phase", phase);
#ifndef USE_QUINTICWALK
    Leph::VectorLabel walkParams;
    Leph::IKWalk::convertParameters(walkParams, params);
    _logs.logData(walkParams);
#endif
    _logs.flush();

    bind->push();
}


void Walk::updateShoot(bool left, const std::string & kickName)
{
    kickMove->set(left, kickName);
}

void Walk::endShoot()
{
    shootingLeft = false;
    shootingRight = false;
}
