#include <cmath>
#include <rhoban_utils/logging/logger.h>
#include <rhoban_utils/angle.h>
#include <services/LocalisationService.h>
#include <services/TeamPlayService.h>
#include <services/RefereeService.h>
#include <services/DecisionService.h>
#include "rhoban_geometry/point.h"
#include "rhoban_utils/logging/logger.h"
#include "moves/ApproachPotential.h"
#include "moves/Walk.h"

static rhoban_utils::Logger logger("ApproachPotential"/*, LoggerDebug*/);

using namespace rhoban_geometry;
using namespace rhoban_utils;

#define STATE_STOPPING          "stopping"
#define STATE_PLACE             "place"
#define STATE_STEPPING          "stepping"
#define STATE_SHOOT             "shoot"

ApproachPotential::Target::Target()
{
}

ApproachPotential::Target::Target(Point position, Angle yaw, bool rightKick, 
        std::string kickName, double tolerance)
    : position(position), yaw(yaw), rightKick(rightKick), 
    kickName(kickName), tolerance(tolerance)
{
}

ApproachPotential::ApproachPotential(Walk *walk)
    : ApproachMove(walk), currentTolerance(0)
{
    Move::initializeBinding();
    initBindings();

    // State
    bind->bindNew("state", STM::state, RhIO::Bind::PushOnly)
        ->comment("Approach STM state");
        
    bind->bindNew("repulsion", repulsion, RhIO::Bind::PullOnly)
        ->defaultValue(0.7);

    bind->bindNew("degsPerMeter", degsPerMeter, RhIO::Bind::PullOnly)
        ->defaultValue(30);

    // Servoing
    bind->bindNew("stepP", stepP, RhIO::Bind::PullOnly)
        ->defaultValue(1);
    bind->bindNew("lateralP", lateralP, RhIO::Bind::PullOnly)
        ->defaultValue(3);
    bind->bindNew("stepI", stepI, RhIO::Bind::PullOnly)
        ->defaultValue(0.0);
    bind->bindNew("lateralI", lateralI, RhIO::Bind::PullOnly)
        ->defaultValue(0.0);
    bind->bindNew("stepPunch", stepPunch, RhIO::Bind::PullOnly)
        ->defaultValue(8);
    bind->bindNew("rotationP", aligner.k_p, RhIO::Bind::PullOnly)
        ->defaultValue(10);
    bind->bindNew("rotationI", aligner.k_i, RhIO::Bind::PullOnly)
        ->defaultValue(0.0);

    // Acceptance
    bind->bindNew("distanceThreshold", distanceThreshold, RhIO::Bind::PullOnly)
        ->defaultValue(0.065)->persisted(true);
    bind->bindNew("angleThreshold", angleThreshold, RhIO::Bind::PullOnly)
        ->defaultValue(9)->persisted(true);

    // Don't walk
    bind->bindNew("dontWalk", dontWalk, RhIO::Bind::PullOnly)
        ->defaultValue(false);

    // Are we using a final last step
    bind->node().newBool("useLastStep")
        ->persisted(false)
        ->defaultValue(true);
}

Angle ApproachPotential::getKickCap()
{
  return ApproachMove::getKickCap()+ Angle(currentTolerance);
}

std::string ApproachPotential::getName()
{
    return "approach_potential";
}

void ApproachPotential::onStart()
{
    ApproachMove::onStart();
    lastFootChoice = 1e6;
    kick_score = 0;
    setState(STATE_PLACE);
    
    stepper.max = walk->maxStep;
    stepper.min = -walk->maxStepBackward;
    lateraler.min = -walk->maxLateral;
    lateraler.max = walk->maxLateral;
    aligner.min = -walk->maxRotation;
    aligner.max = walk->maxRotation;
    
    stepper.reset();
    lateraler.reset();
    aligner.reset();
}

void ApproachPotential::onStop()
{
    ApproachMove::onStop();
    walk->control(false);
    setState(STATE_STOPPING);
}

void ApproachPotential::getControl(const Target &target, const Point &ball,
                        double &x, double &y, double &yaw)
{
    double dist = target.position.getLength();

    // Going directly (to debug the target)
    // logger.log("Target: %f, %f, %f\n", target.position.x, target.position.y, target.yaw.getSignedValue());
    // walk->control(true, target.position.x*100, target.position.y*100, target.yaw.getSignedValue());

    // Magical potential field
    // See Motion/python/potential.py
    double X(0), Y(0);
    double norm2 = sqrt(pow(target.position.x, 2) + pow(target.position.y, 2));
    if (norm2 > 0) {
        X = ball.x*repulsion*pow(pow(ball.x, 2) + pow(ball.y, 2), -1/2*repulsion)*sqrt(pow(target.position.x, 2) + pow(target.position.y, 2))/(pow(ball.x, 2) + pow(ball.y, 2)) - target.position.x*pow(pow(ball.x, 2) + pow(ball.y, 2), -1/2*repulsion)/sqrt(pow(target.position.x, 2) + pow(target.position.y,
 2));
        Y = ball.y*repulsion*pow(pow(ball.x, 2) + pow(ball.y, 2), -1/2*repulsion)*sqrt(pow(target.position.x, 2) + pow(target.position.y, 2))/(pow(ball.x, 2) + pow(ball.y, 2)) - target.position.y*pow(pow(ball.x, 2) + pow(ball.y, 2), -1/2*repulsion)/sqrt(pow(target.position.x, 2) + pow(target.position.y,
 2));
    }

    // XXX: Some below variable should be rhiorized
    Point control(-X, -Y);

    // XXX: dist here may be replaced with the distance following the potential fields
    double P = dist*100;
    control.normalize(P);

    // Applying punch
    if (dist < 0.65) {
        if (control.x > 2) control.x += stepPunch;
        if (control.x < -2) control.x -= stepPunch;
        if (control.y > 2) control.y += stepPunch;
        if (control.y < -2) control.y -= stepPunch;
    }

    // Normalizing using walk max speeds
    if (control.x > walk->maxStep) control.normalize(walk->maxStep);
    if (control.x < -walk->maxStepBackward) control.normalize(walk->maxStepBackward);
    if (control.y > walk->maxLateral) control.normalize(walk->maxLateral);

    double goalCap = atan2(control.y, control.x);
    double ballCap = atan2(ball.y, ball.x);
    double targetCap =  deg2rad(target.yaw.getSignedValue());

    double error = goalCap;
    if (dist < 0.65) {
        // We are near the ball, let's face it
        error = ballCap;
    }
    if (dist < 0.4) {
        // We are near the goal, let's face the goal
        error = targetCap;
    }

    if (dist > 0.65) {
        // Avoiding walking backward when error azimuth is high
        control *= std::max<double>(0, cos(error));
    }

    // Response
    x = control.x;
    y = control.y;
    yaw = error;
}

void ApproachPotential::step(float elapsed)
{
    bind->pull();
    
    stepper.k_p = stepP;
    lateraler.k_p = lateralP;
    
    stepper.k_i = stepI;
    lateraler.k_i = lateralI;
    
    auto loc = getServices()->localisation;

    if (state == STATE_SHOOT) {
        if (!walk->isKicking()) {
            setState(STATE_PLACE);
        }
    }
    
    if (state == STATE_STEPPING) {
        if (!walk->isLastStep()) {
            requestKick();
            setState(STATE_SHOOT);
        }
    }

    if (state == STATE_PLACE) {
        // Target yaw
        auto cap = ApproachMove::getKickCap();

        // Ball position
        auto ball = loc->getBallPosSelf();

        std::vector<Target> targets;
        lastFootChoice += elapsed;
        if (lastFootChoice > 3.0) {
            left = ball.y > 0;
            lastFootChoice = 0;
        }

        auto allowedKicks = getAllowedKicks();

        if (allowedKicks.size()) {
            for (const std::string & name : allowedKicks) {
                for (bool isKickRight : {false, true}) {
                    if (name == "classic" || name == "small") {
                        if (isKickRight != !left) continue;
                    }

                    std::vector<double> toleranceAngles;
                    toleranceAngles.push_back(0);
                    double toleranceStep = 1;// Previously 5
                    double kick_tol_rad = kmc.getKickModel(name).getKickZone().getThetaTol();
                    double maxAlpha = getKickTolerance()- rad2deg(kick_tol_rad);
                    for (double alpha=toleranceStep; alpha< maxAlpha; alpha+=toleranceStep) {
                        toleranceAngles.push_back(-alpha);
                        toleranceAngles.push_back(alpha);
                    }

                    for (const double &tolerance : toleranceAngles) {
                        // Importing wished pos from kick_model
                        const csa_mdp::KickZone & kick_zone =
                            kmc.getKickModel(name).getKickZone();
                        Eigen::Vector3d wishedPos = kick_zone.getWishedPos(isKickRight);
                        // Computing target
                        double targetX = wishedPos(0);
                        double targetY = wishedPos(1);
                        Angle lateralAngle(rad2deg(wishedPos(2)));
                        Angle thetaWished = cap+lateralAngle+Angle(tolerance);

                        Point kickRelativePos(targetX, targetY);
                        Point offset = kickRelativePos.rotation(thetaWished);

                        targets.push_back(Target(ball - offset, thetaWished,
                                    isKickRight, name, tolerance));
                    }
                }
            }

            Target target;
            double bestScore = -1;
            auto ballField = loc->getBallPosField();
            Angle defendTargetDir = Angle(180) - loc->getOurBallToGoalDirSelf();
            //std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;
            //std::cout << "OUR GOAL: " << defendTargetDir << std::endl;

            for (auto t : targets) {
                double cX, cY, cYaw;
                getControl(t, ball, cX, cY, cYaw);
                double score = t.position.getLength();
                score += fabs(rad2deg(cYaw))/degsPerMeter;

                if (ballField.x < 0) {
                    auto defendError = fabs((t.yaw - defendTargetDir).getSignedValue());
                    if (defendError > 30) {
                        score *= 30*defendError;
                    }
                }
                // std::cout << "Score for " << t.kickName << " / " << t.yaw.getSignedValue() <<
                //    " : " << score << std::endl;

                if (bestScore < 0 || score < bestScore) {
                    target = t;
                    bestScore = score;
                }
            }
            
            // std::cout << "Target: " << target.position.x << ", " << target.position.y << ", " << target.yaw.getSignedValue() << std::endl;

            // Setting expectedKick
            expectedKick = target.kickName;
            kickRight = target.rightKick;
            currentTolerance = target.tolerance;

            updateKickScore(elapsed);

            if (kick_score >= 1.0) {
                walk->control(false, 0, 0, 0);
                if (bind->node().getBool("useLastStep")) {
                    //Compute last step displacement
                    Eigen::Vector3d dpose;
                    if (expectedKick != "lateral") {
                        //In case of classic or small forward kick 
                        //rechoose the kick foot for last step
                        Eigen::Vector3d dposeLeft = computeLastStepDelta(expectedKick, false);
                        Eigen::Vector3d dposeRight = computeLastStepDelta(expectedKick, true);
                        //Check which pose is reachable at next step
                        double phase = walk->getPhase();
                        bool isReachableLeft = 
                            (dposeLeft.y() >= 0.0 && phase < 0.5) || (dposeLeft.y() <= 0.0 && phase > 0.5);
                        bool isReachableRight = 
                            (dposeRight.y() >= 0.0 && phase < 0.5) || (dposeRight.y() <= 0.0 && phase > 0.5);
                        //Compute command effort
                        double effortLeft = Eigen::Vector2d(dposeLeft.x(), dposeLeft.y()).norm();
                        double effortRight = Eigen::Vector2d(dposeRight.x(), dposeRight.y()).norm();
                        //Select the minimal step delta which is possible or
                        //the minimal step if none are currently possible
                        bool isSelectedRight = false;
                        if (
                            (!isReachableLeft && !isReachableRight) ||
                            (isReachableLeft && isReachableRight)
                        ) {
                            //None or all are possible, choose the minimal effort one
                            isSelectedRight = (effortLeft > effortRight);
                        } else if (isReachableLeft && !isReachableRight && fabs(dposeLeft.y()) < 0.06) {
                            isSelectedRight = false;
                        } else if (!isReachableLeft && isReachableRight && fabs(dposeRight.y()) < 0.06) {
                            isSelectedRight = true;
                        } else {
                            isSelectedRight = kickRight;
                        }
                        dpose = isSelectedRight ? dposeRight : dposeLeft;
                        kickRight = isSelectedRight;
                        std::cout << "LEPH: name=" << expectedKick << " foot=" << kickRight << std::endl;
                        std::cout << "LEPH: phase:" << phase << std::endl;
                        std::cout << "LEPH: dposeLeft: " << dposeLeft.transpose() << std::endl;
                        std::cout << "LEPH: dposeRight: " << dposeRight.transpose() << std::endl;
                        std::cout << "LEPH: isReachLeft: " << isReachableLeft << std::endl;
                        std::cout << "LEPH: isReachRight: " << isReachableRight << std::endl;
                        std::cout << "LEPH: effortLeft: " << effortLeft << std::endl;
                        std::cout << "LEPH: effortRight: " << effortRight << std::endl;
                        std::cout << "LEPH: selected: " << isSelectedRight << std::endl;
                        std::cout << "LEPH: selectPose: " << dpose.transpose() << std::endl;
                    } else {
                        dpose = computeLastStepDelta(expectedKick, kickRight);
                    }
                    //Security bounds
                    if (dpose.x() > 0.1) dpose.x() = 0.1;
                    if (dpose.x() < -0.05) dpose.x() = -0.05;
                    if (dpose.y() > 0.1) dpose.y() = 0.1;
                    if (dpose.y() < -0.1) dpose.y() = -0.1;
                    if (dpose.z() > 0.8) dpose.z() = 0.8;
                    if (dpose.z() < -0.8) dpose.z() = -0.8;
                    walk->askLastStep(dpose);
                    setState(STATE_STEPPING);
                } else {
                    requestKick();
                    setState(STATE_SHOOT);
                }
            } else {
                // Applying control
                double controlX, controlY, controlYaw;
                getControl(target, ball, controlX, controlY, controlYaw);
                
                controlX = stepper.update(controlX, elapsed);
                controlY = lateraler.update(controlY, elapsed);
                controlYaw = aligner.update(controlYaw, elapsed);
                
                walk->control(!dontWalk, controlX, controlY, controlYaw);
            }
        } else {
            walk->control(false);
        }
    }

    bind->push();
}

void ApproachPotential::enterState(std::string state)
{
    stepper.reset();
    lateraler.reset();
    aligner.reset();
}

void ApproachPotential::exitState(std::string state)
{
}
  
Eigen::Vector3d ApproachPotential::computeLastStepDelta(
    const std::string& kickName, bool isRightFoot)
{
    auto cap = ApproachMove::getKickCap();
    auto loc = getServices()->localisation;
    auto ball = loc->getBallPosSelf();
    Eigen::Vector3d currentEgoToBall(ball.x, ball.y, cap.getSignedValue()*M_PI/180.0);
    double phase = walk->getPhase();
    Eigen::Vector4d order = walk->getRawOrder();

    //Predict egocentric ball pose 
    //at the end of current step.
    //Compute the remaining delta pose
    //to travel to the end of 
    //current step (half cycle).
    double ratio;
    if (phase < 0.5) {
        ratio = 2.0*phase;
    } else {
        ratio = 2.0*(phase-0.5);
    }
    const double odometryTransCoef = 1.1;
    const double odometryRotCoef = 1.0;
    Eigen::Vector3d egoAtNext(
        odometryTransCoef*(1.0-ratio)*order.x(),
        odometryTransCoef*(1.0-ratio)*order.y(),
        odometryRotCoef*(1.0-ratio)*order.z());
    double vectX = currentEgoToBall.x() - egoAtNext.x();
    double vectY = currentEgoToBall.y() - egoAtNext.y();
    double angle = currentEgoToBall.z() - egoAtNext.z();
    double aa = egoAtNext.z();
    double vectInSrcX = vectX*cos(-aa) - vectY*sin(-aa);
    double vectInSrcY = vectX*sin(-aa) + vectY*cos(-aa);
    Eigen::Vector3d nextEgoToBall(vectInSrcX, vectInSrcY, angle);
    
    //Compute last step displacement
    Eigen::Vector3d targetEgoToBall = kmc.getKickModel(kickName)
        .getKickZone().getWishedPos(isRightFoot);
    targetEgoToBall.z() *= -1.0;
    Eigen::Vector3d targetBallToEgo;
    double tmpA = -targetEgoToBall.z();
    targetBallToEgo.x() = -targetEgoToBall.x()*std::cos(tmpA) + targetEgoToBall.y()*sin(tmpA);
    targetBallToEgo.y() = -targetEgoToBall.x()*std::sin(tmpA) - targetEgoToBall.y()*cos(tmpA);
    targetBallToEgo.z() = tmpA;
    Eigen::Vector3d dpose(
        nextEgoToBall.x(), 
        nextEgoToBall.y(), 
        nextEgoToBall.z() + targetBallToEgo.z());
    dpose.x() += 
        targetBallToEgo.x()*std::cos(nextEgoToBall.z()) - 
        targetBallToEgo.y()*std::sin(nextEgoToBall.z());
    dpose.y() += 
        targetBallToEgo.x()*std::sin(nextEgoToBall.z()) + 
        targetBallToEgo.y()*std::cos(nextEgoToBall.z());

    std::cout << "LEPH|: phase: " << phase << std::endl;
    std::cout << "LEPH|: order:     " << order.transpose() << std::endl;
    std::cout << "LEPH|: ratio: " << ratio << std::endl;
    std::cout << "LEPH|: egoAtNext: " << egoAtNext.transpose() << std::endl;
    std::cout << "LEPH|: currentToBall: " << currentEgoToBall.transpose() << std::endl;
    std::cout << "LEPH|: nextToBall   : " << nextEgoToBall.transpose() << std::endl;
    std::cout << "LEPH|: dpose: " << dpose.transpose() << std::endl;

    return dpose;
}

