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

    bind->bindNew("degsPerMeter", degsPerMeter, RhIO::Bind::PullOnly)
        ->defaultValue(200)->persisted(true);

    // Ball position target
    bind->bindNew("targetX", targetX, RhIO::Bind::PullOnly)
        ->defaultValue(0.1)->persisted(true);
    bind->bindNew("targetY", targetY, RhIO::Bind::PullOnly)
        ->defaultValue(0.08)->persisted(true);

    // Servoing
    bind->bindNew("stepP", stepP, RhIO::Bind::PullOnly)
        ->defaultValue(1)->persisted(true);
    bind->bindNew("stepPunch", stepPunch, RhIO::Bind::PullOnly)
        ->defaultValue(0)->persisted(true);
    bind->bindNew("rotationP", rotationP, RhIO::Bind::PullOnly)
        ->defaultValue(10)->persisted(true);

    // Acceptance
    bind->bindNew("distanceThreshold", distanceThreshold, RhIO::Bind::PullOnly)
        ->defaultValue(0.065)->persisted(true);
    bind->bindNew("angleThreshold", angleThreshold, RhIO::Bind::PullOnly)
        ->defaultValue(9)->persisted(true);

    // Don't walk
    bind->bindNew("dontWalk", dontWalk, RhIO::Bind::PullOnly)
        ->defaultValue(false);
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
    setState(STATE_PLACE);
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
        X = 0.67*ball.x*pow(pow(ball.x, 2) + pow(ball.y, 2), -1.335)*sqrt(pow(target.position.x, 2) + pow(target.position.y, 2)) - target.position.x*pow(pow(ball.x, 2) + pow(ball.y, 2), -0.335)/sqrt(pow(target.position.x, 2) + pow(target.position.y, 2));
        Y = 0.67*ball.y*pow(pow(ball.x, 2) + pow(ball.y, 2), -1.335)*sqrt(pow(target.position.x, 2) + pow(target.position.y, 2)) - target.position.y*pow(pow(ball.x, 2) + pow(ball.y, 2), -0.335)/sqrt(pow(target.position.x, 2) + pow(target.position.y, 2));
    }

    // XXX: Some below variable should be rhiorized
    Point control(-X, -Y);

    // XXX: dist here may be replaced with the distance following the potential fields
    double P = dist*stepP*100;
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
    if (dist < 0.2) {
        // We are near the goal, let's face the goal
        error = targetCap;
    }

    if (dist > 0.65) {
        // Avoiding walking backward when error azimuth is high
        control *= std::max<double>(0, cos(error));
    }

    // Response
    x = control.x*stepP;
    y = control.y*stepP;
    yaw = error*rotationP;
}

void ApproachPotential::step(float elapsed)
{
    bind->pull();
    auto loc = getServices()->localisation;

    if (state == STATE_SHOOT) {
        if (!walk->isKicking()) {
            setState(STATE_PLACE);
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
                score += fabs(cYaw)/degsPerMeter;

                if (ballField.x < 0) {
                    auto defendError = fabs((t.yaw - defendTargetDir).getSignedValue());
                    if (defendError > 30) {
                        score *= 30*defendError;
                    }
                }
                //std::cout << "Score for " << t.kickName << " / " << t.yaw.getSignedValue() <<
                //    " : " << score << std::endl;

                if (bestScore < 0 || score < bestScore) {
                    target = t;
                    bestScore = score;
                }
            }

            // Setting expectedKick
            expectedKick = target.kickName;
            kickRight = target.rightKick;
            currentTolerance = target.tolerance;

            updateKickScore(elapsed);

            if (kick_score >= 1.0) {
                requestKick();
                walk->control(!dontWalk, 0, 0, 0);
                setState(STATE_SHOOT);
            } else {
                // Applying control
                double controlX, controlY, controlYaw;
                getControl(target, ball, controlX, controlY, controlYaw);
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
}

void ApproachPotential::exitState(std::string state)
{
}
