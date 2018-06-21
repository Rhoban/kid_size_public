#include <math.h>
#include <algorithm>
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
#include <rhoban_utils/stats/stats.h>
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
    : kickMove(kickMove), ratioHistory(0.5)
{
    _params = _engine.getParameters();
    _lastStepParams = _engine.getParameters();
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

    bind->bindNew("freq", _params("freq"), RhIO::Bind::PullOnly)
        ->comment("Walk frequency")->persisted(true)
        ->defaultValue(2)->minimum(0.1)->maximum(10.0)
        ;
    bind->bindNew("supportPhaseRatio", _params("doubleSupportRatio"), RhIO::Bind::PullOnly)
        ->comment("Support phase ratio")->persisted(true)
        ->defaultValue(0.0)->minimum(0.0)->maximum(1.0);

    // Offsets
    bind->bindNew("footYOffset", _footYOffset, RhIO::Bind::PullOnly)
        ->comment("Front Y offset")->persisted(true)
        ->defaultValue(0.01)->minimum(-0.1)->maximum(0.1)
        ;
    bind->bindNew("trunkYOffset", trunkYOffset, RhIO::Bind::PullOnly)
        ->comment("Trunk Y offset")->persisted(true)
        ->defaultValue(0.00)->minimum(-0.1)->maximum(0.1)
        ;
    bind->bindNew("trunkZOffset", _trunkZOffset, RhIO::Bind::PullOnly)
        ->comment("Trunk Z offset")->persisted(true)
        ->defaultValue(0.02)->minimum(0.0)->maximum(0.2)
        ;

    // Splines
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
    
    bind->bindNew("startPhase", startPhase, RhIO::Bind::PullOnly)
        ->comment("Start phase")->persisted(true)
        ->defaultValue(0.0)->minimum(0.0)->maximum(1.0)
        ;

    // Swing
    bind->bindNew("swingGain", swingGain, RhIO::Bind::PullOnly)
        ->comment("Swing gain")->persisted(true)
        ->defaultValue(0.0)->minimum(-0.1)->maximum(0.1)
        ;
    bind->bindNew("swingPhase", swingPhase, RhIO::Bind::PullOnly)
        ->comment("Swing phase")->persisted(true)
        ->defaultValue(0.0)->minimum(0.0)->maximum(1.0)
        ;
    bind->bindNew("swingPause", _params("trunkPause"), RhIO::Bind::PullOnly)
        ->comment("Swing pause")->persisted(true)
        ->defaultValue(0.0)->minimum(0.0)->maximum(0.5)
        ;

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

    //Last step variables
    bind->node().newInt("lastStepAsk")
        ->comment("0: no last step. 1: left foot. 2: right foot.")
        ->defaultValue(0);
    bind->node().newFloat("lastStepForward")
        ->defaultValue(0.0);
    bind->node().newFloat("lastStepLateral")
        ->defaultValue(0.0);
    bind->node().newFloat("lastStepTurn")
        ->defaultValue(0.0);

    // Shoot warmup and cool down
    bind->bindNew("cooldown", cooldown, RhIO::Bind::PullOnly)
        ->defaultValue(1.0)->comment("Cooldown duration [s]")->persisted(false);
    bind->bindNew("warmup", warmup, RhIO::Bind::PullOnly)
        ->defaultValue(0.75)->comment("Warmup [s]")->persisted(false);
        
    bind->bindNew("paramsStepGain", _orders.x(), RhIO::Bind::PushOnly)
        ->comment("IKWalk really used step parameters");
    bind->bindNew("paramsLateralGain", _orders.y(), RhIO::Bind::PushOnly)
        ->comment("IKWalk really used lateral parameters");
    bind->bindNew("paramsTurnGain", _orders.z(), RhIO::Bind::PushOnly)
        ->comment("IKWalk really used turn parameters");

    bind->bindNew("pressureYStdThresholdWarmup", pressureYStdThresholdWarmup, RhIO::Bind::PullOnly)
        ->defaultValue(0.04);
    bind->bindNew("pressureYStdThresholdCooldown", pressureYStdThresholdCooldown, RhIO::Bind::PullOnly)
        ->defaultValue(0.02);
    bind->bindNew("pressureYStd", pressureYStd, RhIO::Bind::PushOnly);
    bind->bindNew("pressureY", pressureY, RhIO::Bind::PushOnly);
    
    // Speed limits
    bind->bindNew("maxRotation", maxRotation, RhIO::Bind::PullOnly)
        ->defaultValue(12.0);

    bind->bindNew("maxStep", maxStep, RhIO::Bind::PullOnly)
        ->defaultValue(40.0);

    bind->bindNew("maxStepBackward", maxStepBackward, RhIO::Bind::PullOnly)
        ->defaultValue(20.0);

    bind->bindNew("maxLateral", maxLateral, RhIO::Bind::PullOnly)
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

    bind->bindNew("gkMustRaise", gkMustRaise, RhIO::Bind::PullOnly)
            ->comment("goal keeper flag: walk must raise")->defaultValue(false);
    bind->bindNew("gkMustBlock", gkMustBlock, RhIO::Bind::PullOnly)
            ->comment("goal keeper flag: walk must be blocked")->defaultValue(false);

#ifdef MODE_NOMINAL
    bind->node().newFloat("guessPhase");
    bind->node().newFloat("nominalScore");
    bind->node().newFloat("phaseMaxV")
        ->defaultValue(0.1);
    bind->node().newBool("nominalError");
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
    return Eigen::Vector4d(
        _orders.x(), 
        _orders.y(), 
        _orders.z(),
        _isEnabled);
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
        
void Walk::askLastStep(const Eigen::Vector3d& deltaPose)
{
    if (!_isEnabled || isLastStep()) {
        return;
    }

    bool isFootLeft = true;
    if (deltaPose.y() > 0.0) {
        isFootLeft = true;
    } else if (deltaPose.y() < 0.0) {
        isFootLeft = false;
    } else if (deltaPose.z() >= 0.0) {
        isFootLeft = true;
    } else {
        isFootLeft = false;
    }

    if (isFootLeft) {
        bind->node().setInt("lastStepAsk", 1);
    } else {
        bind->node().setInt("lastStepAsk", 2);
    }
    bind->node().setFloat("lastStepForward", deltaPose.x());
    bind->node().setFloat("lastStepLateral", deltaPose.y());
    bind->node().setFloat("lastStepTurn", deltaPose.z());
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
        
bool Walk::isLastStep() const
{
    return 
        (_lastStepPhase >= 0.0) ||
        (bind->node().getInt("lastStepAsk") != 0);
}

void Walk::onStart()
{
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
    ratioHistory.clear();
    bind->pull();
    lastWalkEnable = false;
    walkEnable = false;
    walkEnableTarget = false;
    walkEnableTimeSinceChange = 0;
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
    pressureY = 0;
    pressureYStd = 0;
    t = 0;
    ratioHistory.clear();
    _lastStepPhase = -1.0;
    _lastStepCount = 0;

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

    //security for arms. TODO should be a init command.
    setTorqueLimit("left_shoulder_pitch", 0.2);
    setTorqueLimit("right_shoulder_pitch", 0.2);
    setTorqueLimit("left_shoulder_roll", 0.2);
    setTorqueLimit("right_shoulder_roll", 0.2);


    initElbowOffsetValue=RhIO::Root.getFloat("/moves/walk/elbowOffset");
    initArmsRollValue=RhIO::Root.getFloat("/moves/walk/armsRoll");
    initTrunkZOffsetValue=0.02;//RhIO::Root.getFloat("/moves/walk/trunkZOffset");
    
    
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

    _params("footDistance") = 2.0*_footYOffset + _footDistance;
    _params("trunkSwing")     = 
        (factor*_params("trunkSwing")*std::fabs(_params("footDistance")) + (1-factor)*targetSwingGain)/std::fabs(_params("footDistance"));
    _params("trunkPhase")     = factor*_params("trunkPhase")        + (1-factor)*targetSwingPhase;
    _params("trunkPitch")     = factor*_params("trunkPitch")        + (1-factor)*targetTrunkPitch;
    _params("trunkXOffset")   = factor*_params("trunkXOffset")      + (1-factor)*targetTrunkXOffset;
    _params("trunkYOffset")   = factor*_params("trunkYOffset")      + (1-factor)*targetTrunkYOffset;
    _params("trunkHeight")    = _trunkHeight - _trunkZOffset;
}


static float getLinear(float x,float x1, float y1,float x2, float y2){
  if (x1==x2){
    return y2;
  }
  if (y1==y2) return y1;
  float a=(y1-y2)/(x1-x2);
  return x*a+y2-x2*a;
}

void Walk::step(float elapsed)
{
    static float t;
    static float tgk=-1,tgk2=-1;
    auto &decision = getServices()->decision;
    auto &model = getServices()->model;
    
    t += elapsed;

    bind->pull();

    if (gkMustBlock){
      return;
    }
    
    if (gkMustRaise){
      if (tgk<0) tgk=0;
      tgk+=elapsed;
      if (_trunkZOffset>initTrunkZOffsetValue){
	double x=RhIO::Root.getFloat("/moves/goal_keeper/stopMoveTime");

	double z;
	if (tgk>x)
	  z=_trunkZOffset-0.1;
	else
	  z=getLinear(tgk+elapsed,tgk,_trunkZOffset,x,initTrunkZOffsetValue);
	if (z<initTrunkZOffsetValue) z=initTrunkZOffsetValue;
	RhIO::Root.setFloat("/moves/walk/trunkZOffset", z);
      }
      else {
	tgk=-1;
	if (tgk2<0) tgk2=elapsed;
	else tgk2+=elapsed;
	if (tgk2>1){ // leave 1s of cool down
	  tgk2=-1;
	  RhIO::Root.setFloat("/moves/walk/elbowOffset", initElbowOffsetValue);
	  RhIO::Root.setFloat("/moves/walk/armsRoll", initArmsRollValue);
	  RhIO::Root.setBool("/moves/walk/gkMustRaise",false);
	}
      }
    }
    
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
    smoothing = smoothing*smoothCommands + (walkEnableTarget ? 1 : 0)*(1.0-smoothCommands);
    bool newStep = isNewStep();
    if (!forbidOrders && newStep) {
        double tmpMaxDStepByCycle = maxDStepByCycle;
        double tmpMaxDLatByCycle = maxDLatByCycle;
        double tmpMaxDTurnByCycle = maxDTurnByCycle;
        
        // We accept way more deceleration than acceleration
        if (smoothingStep > step) tmpMaxDStepByCycle *= 5;
        if (smoothingLateral > lateral) tmpMaxDLatByCycle *= 5;
        if (smoothingTurn > turn) tmpMaxDTurnByCycle *= 5;
        
        VariationBound::update(smoothingStep, step, tmpMaxDStepByCycle, 1);
        VariationBound::update(smoothingLateral, lateral, tmpMaxDLatByCycle, 1);
        VariationBound::update(smoothingTurn, turn, tmpMaxDTurnByCycle, 1);
    }
    bool isEnabled = (smoothing > 0.95);
    
    // Computing y pressure std
    ratioHistory.pushValue(t, pressureY);
    pressureYStd = 0;
    auto historyValues = ratioHistory.getValues();
    std::vector<double> values;
    for (auto value : historyValues) {
        values.push_back(value.second);
    }
    pressureYStd = standardDeviation(values);
    bool isStableWarmup = pressureYStd < pressureYStdThresholdWarmup;
    bool isStableCooldown = pressureYStd < pressureYStdThresholdCooldown;

    
    if (walkEnable != walkEnableTarget) {
        walkEnableTimeSinceChange += elapsed;
        if ((!walkEnable && walkEnableTimeSinceChange > 0.3) ||
            (walkEnable && isStableCooldown)) {
            walkEnableTarget = walkEnable;
        }
    } else {
        walkEnableTimeSinceChange = 0;
    }


    // Setting real parameters used gains
    _isEnabled = (smoothing > 0.5);
    if (forbidOrders) {
        _orders.setZero();
    }
    else if (newStep) {
        _orders.x() = (stepTrim+smoothingStep)/1000.0;
        _orders.y() = (lateralTrim+smoothingLateral)/1000.0;
        _orders.z() = deg2rad(turnTrim+smoothingTurn);
    }
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

    model->setReadBaseUpdate(_engine.isEnabled());
    
    if (!_engine.isEnabled()) {
        waitT += elapsed;
    }

    if (shootingLeft || shootingRight) {
        if (isWarmingUp) {
	  if (isStableWarmup && waitT > warmup && !decision->freezeKick && (gkMustRaise==false)) {
                startMove("kick", 0.0);
                isWarmingUp = false;
                shootT = 0;
            }
        } else if (isCoolingDown) {
            if (isStableCooldown && waitT > cooldown) {
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

        _isEnabled = false;
        _orders.setZero();
    }
    
    auto& pressureLeft = getScheduler()->getManager()->dev<RhAL::PressureSensor4>("left_pressure");
    auto& pressureRight = getScheduler()->getManager()->dev<RhAL::PressureSensor4>("right_pressure");
    double total = pressureLeft.getWeight() + pressureRight.getWeight();
    pressureY = 0;
    if (total > 0) {
        pressureY = 
            (
            pressureLeft.getWeight()*(pressureLeft.getY() + 0.07) +
            pressureRight.getWeight()*(pressureRight.getY() - 0.07)
            )/total
            ;
    }

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

    //Check if a last step is asked
    int isAskLast = bind->node().getInt("lastStepAsk");
    if (_lastStepPhase < 0.0 && isEnabled && isAskLast != 0) {
        //Start the last step
        _lastStepPhase = _engine.getPhase();
        _lastStepCount = 0;
    }

    float prevPhase = phase;
    //Control the walk engine
    if (_lastStepPhase >= 0.0) {
        //Last step case
        //Count half walk cycle
        //and walk in place during waiting
        if (
            _lastStepCount%2 == 0 &&
            ((_lastStepPhase >= 0.0 && 
            _lastStepPhase < 0.5 && 
            _engine.getPhase() > 0.5) ||
            (_lastStepPhase >= 0.5 && 
            _lastStepPhase < 1.0 && 
            _engine.getPhase() < 0.5))
        ) {
            _lastStepCount++;
        }
        if (
            _lastStepCount%2 == 1 &&
            ((_lastStepPhase >= 0.0 && 
            _lastStepPhase < 0.5 && 
            _engine.getPhase() < 0.5) ||
            (_lastStepPhase >= 0.5 && 
            _lastStepPhase < 1.0 && 
            _engine.getPhase() > 0.5))
        ) {
            _lastStepCount++;
        }
        //Retrieve last step displacement
        Eigen::Vector3d lastOrder(
            bind->node().getFloat("lastStepForward"),
            bind->node().getFloat("lastStepLateral"),
            bind->node().getFloat("lastStepTurn"));
        //Select when to make the step according 
        //to chosen foot and current walk phase
        bool isFinished = false;
        if (isAskLast == 1 && _lastStepPhase > 0.5) {
            if (_lastStepCount == 0) {
                _engine.setOrders(Eigen::Vector3d(0.0, 0.0, 0.0), true);
            } else if (_lastStepCount == 1) {
                _engine.setOrders(lastOrder, true);
            } else {
                isFinished = true;
            }
        } else if (isAskLast == 1 && _lastStepPhase < 0.5) {
            if (_lastStepCount == 0) {
                _engine.setOrders(lastOrder, true);
            } else {
                isFinished = true;
            }
        } else if (isAskLast == 2 && _lastStepPhase > 0.5) {
            if (_lastStepCount == 0) {
                _engine.setOrders(lastOrder, true);
            } else {
                isFinished = true;
            }
        } else if (isAskLast == 2 && _lastStepPhase < 0.5) {
            if (_lastStepCount == 0) {
                _engine.setOrders(Eigen::Vector3d(0.0, 0.0, 0.0), true);
            } else if (_lastStepCount == 1) {
                _engine.setOrders(lastOrder, true);
            } else {
                isFinished = true;
            }
        } 
        //Stop the walk when the last step is performed
        if (isFinished) {
            _lastStepPhase = -1.0;
            _lastStepCount = 0;
            bind->node().setInt("lastStepAsk", 0);
            _isEnabled = false;
            smoothing = 0.0;
            walkEnableTarget = false;
            walkEnable = false;
            bind->node().setBool("walkEnable", false);
            _engine.setOrders(Eigen::Vector3d(0.0, 0.0, 0.0), false);
        }
    } else {
        //Normal step case
	if (gkMustRaise) _isEnabled=false;
        _engine.setParameters(_params);
        _engine.setOrders(_orders, _isEnabled);
    }
    if (!_securityEnabled) {
        _engine.update(elapsed);
    }
    phase = _engine.getPhase();
    bool isSuccess = _engine.assignModel(model->goalModel());
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
                if (pressureY > securityTreshold) {
                    securityBlock = true;
                }
            }
            // Left foot security
            double p = 0.5+securityPhase;
            if (p > 1) p -= 1;
            if (phasePassed(prevPhase, phase, p)) {
                if (pressureY < -securityTreshold) {
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

        _securityEnabled = securityBlock;

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

