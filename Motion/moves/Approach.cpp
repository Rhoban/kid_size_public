#include <cmath>
#include <rhoban_utils/logging/logger.h>
#include <rhoban_utils/angle.h>
#include <services/LocalisationService.h>
#include <services/RefereeService.h>
#include "rhoban_geometry/point.h"
#include "rhoban_utils/logging/logger.h"
#include "moves/Approach.h"
#include "moves/Walk.h"
#include "moves/Head.h"

using namespace rhoban_geometry;
using namespace rhoban_utils;
using csa_mdp::KickZone;

static rhoban_utils::Logger logger("Approach"/*, LoggerDebug*/);

static double angleBetween(const Point &pt1, const Point &pt2)
{
  double nv1 = pt1.getLength();
  double nv2 = pt2.getLength();
  if (nv1 < 1e-9 || nv2 < 1e-9) {
      return 0;
  }
  double x = Point::dotProduct(pt1, pt2) / (nv1 * nv2);
  if (x > 1.0) { x = 1.0;}
  if (x < -1.0) { x = -1.0;}
  return acos(x);
}

#define DIRECTION_A        1
#define DIRECTION_B        2

#define STATE_FAR          "far"
#define STATE_ROTATE       "rotate"
#define STATE_NEAR         "near"
#define STATE_SHOOT        "shoot"
#define STATE_STOPPING     "stopping"

Approach::Approach(Walk *walk, Head *head)
    : ApproachMove(walk), head(head)
{
    Move::initializeBinding();
    initBindings();
    // State
    bind->bindNew("state", STM::state, RhIO::Bind::PushOnly)
        ->comment("Approach STM state");
    // Align
    bind->bindNew("farAlignerP", farAlignerP, RhIO::Bind::PullOnly)
        ->defaultValue(0.4)->persisted(true);
    bind->bindNew("rotateAlignerP", rotateAlignerP, RhIO::Bind::PullOnly)
        ->defaultValue(0.4)->persisted(true);
    bind->bindNew("nearAlignerP", nearAlignerP, RhIO::Bind::PullOnly)
        ->defaultValue(0.2)->persisted(true);
    bind->bindNew("rotateLateralP", rotateLateralP, RhIO::Bind::PullOnly)
        ->defaultValue(-0.6)->persisted(true);
    bind->bindNew("nearLateralP", nearLateralP, RhIO::Bind::PullOnly)
        ->defaultValue(0.6)->persisted(true);
    // Walk to the ball

    bind->bindNew("stepP", stepper.k_p, RhIO::Bind::PullOnly)
        ->defaultValue(1)->persisted(true);
    // Radius
    bind->bindNew("radius", radius)
      ->defaultValue(0.45)->minimum(0.0)->maximum(1.0);
    // Time interval between shoots
    bind->bindNew("shootInterval", shootInterval, RhIO::Bind::PullOnly)
        ->defaultValue(3)->persisted(true);
    
    // Centering corner distance
    bind->bindNew("centeringCorner", centeringCorner, RhIO::Bind::PullOnly)
        ->defaultValue(2.50)->persisted(true);

    bind->bindNew("elapsedLastShoot", elapsedLastShoot, RhIO::Bind::PushOnly);
    bind->bindNew("timeSinceNear", timeSinceNear, RhIO::Bind::PushOnly);
    bind->bindNew("ballDistance", ballDistance, RhIO::Bind::PushOnly);
    bind->bindNew("ballAzimuth", ballAzimuth, RhIO::Bind::PushOnly);
    bind->bindNew("ballX", ballX, RhIO::Bind::PushOnly);
    bind->bindNew("ballY", ballY, RhIO::Bind::PushOnly);
    bind->bindNew("goalQ", goalQ, RhIO::Bind::PushOnly);
    bind->bindNew("goalAzimuth", goalAzimuth, RhIO::Bind::PushOnly);
    bind->bindNew("goalLeftAzimuth", goalLeftAzimuth, RhIO::Bind::PushOnly);
    bind->bindNew("goalRightAzimuth", goalRightAzimuth, RhIO::Bind::PushOnly);

    // Idiot mode
    bind->bindNew("idiot", idiot, RhIO::Bind::PullOnly)
        ->comment("Attack with only the magnetometer")->defaultValue(false)
        ->persisted(true);
    // Penalty mode
    bind->bindNew("penalty", penalty, RhIO::Bind::PullOnly)
        ->comment("Attack with only the magnetometer")->defaultValue(false)
        ->persisted(true);

    // Lateral kick treshold
    bind->bindNew("lateralKickTreshold", lateralKickTreshold, RhIO::Bind::PullOnly)
        ->comment("Choose the range where we will do lateral kick")->defaultValue(75)->persisted(true);

    bind->bindNew("enableLateralKick", enableLateralKick, RhIO::Bind::PullOnly)
        ->comment("Enable the lateral kicks")->defaultValue(true)->persisted(true);
    bind->bindNew("placementMargin", placementMargin, RhIO::Bind::PullOnly)
        ->comment("Distance (m) to the target to enter rotate")
        ->defaultValue(0.3)->persisted(true);

    bind->bindNew("theta", theta, RhIO::Bind::PushOnly);
    bind->bindNew("targetPX", targetPX, RhIO::Bind::PushOnly);
    bind->bindNew("targetPY", targetPY, RhIO::Bind::PushOnly);
}
        
void Approach::setPenalty(bool enable)
{
    penalty = enable;
    bind->node().setBool("penalty", enable);
}

std::string Approach::getName()
{
    return "approach";
}

void Approach::onStart()
{
    ApproachMove::onStart();

    timeSinceNear = 3600.0;
    setState(STATE_FAR);
    walk->control(false);
    elapsedLastShoot = 1e6;
    nbShoot = 0;
    clearing = false;

    if (penalty) {
        penaltySide = (time(NULL)&1) ? -1 : 1;
    }
}

void Approach::onStop()
{
    ApproachMove::onStop();
    walk->control(false);
    setState(STATE_STOPPING);
}

void Approach::step(float elapsed)
{
    elapsedLastShoot += elapsed;
    auto loc = getServices()->localisation;
    bind->pull();

    // Getting data from localisation
    goalAzimuth = getKickCap().getSignedValue();
    if (penalty) {
      if (penaltySide < 0) {
        goalAzimuth = loc->getPenaltyLeftGoalCap();
      } else {
        goalAzimuth = loc->getPenaltyRightGoalCap();
      }
    }
    auto ballPos = loc->getBallPosSelf();
    goalLeftAzimuth = loc->getLeftGoalCap();
    goalRightAzimuth = loc->getRightGoalCap();
    ballAzimuth = ballPos.getTheta().getSignedValue();
    ballDistance = ballPos.getLength();
    ballX = ballPos.x;
    ballY = ballPos.y;

    // Setting min & max from parameters
    aligner.min = -walk->maxRotation;
    aligner.max = walk->maxRotation;
    stepper.min = -walk->maxStepBackward;
    stepper.max = walk->maxStep;
    lateraler.min = -walk->maxLateral;
    lateraler.max = walk->maxLateral;

    t += elapsed;
    timeSinceNear += elapsed;

    // XXX: To be improved (by adding hysteresis)
    // XXX: goalQ is handled in behavior so it might not be needed anymore
    goalQ = loc->fieldQ;

    float targetAzimuth = getKickCap().getSignedValue();

    // Choosing kick except in state_near
    bool lateralAllowed = enableLateralKick && isKickAllowed("lateral");
    if (state != STATE_NEAR) {
      if (lateralAllowed && std::fabs(targetAzimuth) > lateralKickTreshold) {
        expectedKick = "lateral";
        kickRight = targetAzimuth > 0;
      } else {
        // We are aiming to kick forward
        if (isKickAllowed("classic")) {
          expectedKick = "classic";
        } else {
          expectedKick = "small";
        }
        // While we are not in near, left or right kick doesn't matter
        kickRight = true;
      }
    }

    const KickZone & kick_zone = kmc.getKickModel(expectedKick).getKickZone();
    Eigen::Vector3d wishedPos = kick_zone.getWishedPos(kickRight);
    double targetX     = wishedPos(0);
    double footTargetY = wishedPos(1);
    double wished_dir = rhoban_utils::rad2deg(wishedPos(2));
    double xRange =  kick_zone.getXRange();
    double yRange =  kick_zone.getYRange();
    targetAzimuth += wished_dir;

    bool goodAlignGoalCenter = (fabs(targetAzimuth) < 15);
    bool goodAlignGoalLeft = (goalLeftAzimuth > 8);
    bool goodAlignGoalRight = (goalRightAzimuth < -8);
    bool goodAlignGoal;
    if (clearing) {
        goodAlignGoal = (fabs(targetAzimuth) < 35);
    } else {
        goodAlignGoal = goodAlignGoalCenter;
    }
    // If we are not in idiot mode and we are not using KickController:
    // - also accept alignement if we have margin for both posts
    if (!idiot && !useKickController()) {
      goodAlignGoal = goodAlignGoal || (goodAlignGoalLeft && goodAlignGoalRight);
    }
    bool goodAlignBall = (fabs(ballAzimuth) < 15);

    if (state == STATE_FAR || state == STATE_ROTATE || state == STATE_NEAR) {
        walk->control(true);
    }

    if (state == STATE_FAR) {
        float goalT = deg2rad(goalAzimuth);
        float bigRadius = 2.0*radius;
        // Ball
        Point B(ballX, ballY);
        // Vector from ball to target for forward kick
        Point V(cos(goalT+M_PI)*bigRadius, sin(goalT+M_PI)*bigRadius);
        // Target to kick forward
        Point T = B+V;
        // Targets for lateral kicks
        Point IA = B+V.rotation(80);
        Point IB = B+V.rotation(-80);
        // Angle between the ball-to-goal vector and the robot
        theta = Angle(ballAzimuth + 180 - goalAzimuth).getSignedValue();
        Point target = T;

        if (fabs(theta) < 75) {
            // We must avoid the ball radius and walk to lateral targets
            if (IA.getLength() < IB.getLength()) {
                target = IA;
            } else {
                target = IB;
            }
        } else {
            if (enableLateralKick) {
                // Go toward the nearest point
                if (IA.getLength() < target.getLength()) target = IA;
                if (IB.getLength() < target.getLength()) target = IB;
            }
        
            float a = angleBetween(target, B);

            if (target.getLength() < placementMargin || a > deg2rad(60)) {
                logger.log("FAR: Placement reached, going to near");
                setState(STATE_ROTATE);
            }
        }

        // For debug
        targetPX = target.x;
        targetPY = target.y;

        // Azimuth error
        float errorAzimuth = atan2(target.y, target.x);

        // We aims at the target
        stepper.update(target.x);

        // Walk toward thetarget when we face it
        float stepCos = cos(errorAzimuth);
        if (stepCos < 0) stepCos = 0;

        // Keeping the target aligned
        aligner.k_p = farAlignerP;
        aligner.update(rad2deg(errorAzimuth));

        // Walking to the target
        walk->control(true, stepCos*stepper.output, 0, aligner.output);

        if (ballDistance < 1.2*radius) {
            logger.log("FAR: Going to rotate, we are close enough");
            setState(STATE_ROTATE);
        }
    } else if (state == STATE_ROTATE) {
        timeSinceNear = 0.0;
        // Keeping at given radius
        stepper.update(ballDistance-radius);

        // Keeping aligned to the ball
        aligner.k_p = rotateAlignerP;
        aligner.update(ballAzimuth);

        // Using lateral steps to get the right alignment
        lateraler.k_p = rotateLateralP;
        lateraler.update(targetAzimuth);

        // Sending orders to walk
        walk->control(true, stepper.output, lateraler.output, aligner.output);

        if (ballDistance > 2.5*radius) {
            logger.log("ROTATE: Ball is too far, going to far");
            setState(STATE_FAR);
        }
        if (goodAlignGoal && goodAlignBall && ballDistance < 1.2*radius) {
            logger.log("ROTATE: Good alignment, going to near");
            setState(STATE_NEAR);
        }
    } else if (state == STATE_NEAR) {
        timeSinceNear = 0.0;
        float turn = 0;
        // Keeping aligned with goals only if goal is not aligned anymore
        if (!goodAlignGoal) {
            aligner.k_p = rotateAlignerP / 5;
            aligner.update(targetAzimuth);
            turn = aligner.output;
        } else {
            turn = 0;
        }

        // Errors
        float errX = ballX-targetX;
        float errY = ballY-footTargetY;
        float stepperErrX = errX;

        // Avoiding forward/backward step when too much error on Y
        if (fabs(errY) > 12.5) {
            stepperErrX = 0;
        }

        // Trying to go in a zone where the ball can be kicked
        lateraler.k_p = nearLateralP;
        lateraler.update(errY);

        stepper.update(stepperErrX);

        // Sending order to walk
        walk->control(true, stepper.output, lateraler.output, turn);

        if (fabs(ballY) > radius) {
            logger.log("NEAR: Ball is too aside, goind to rotate");
            setState(STATE_ROTATE);
        }
        if (ballDistance > 1.8*radius) {
            logger.log("NEAR: Ball is too far, going to far");
            setState(STATE_FAR);
        }
        // Manually update KickScore (we use a custom theta tol)
        if (fabs(errX)<xRange && fabs(errY)<yRange) {
            kick_score += elapsed * kick_gain;
        } else {
            kick_score -= elapsed * no_kick_gain;
        }
        // Bounding kick_score values
        kick_score = std::max(0.0, std::min(1.0, kick_score));
        if (kick_score >= 1.0 && elapsedLastShoot > shootInterval) {
            logger.log("NEAR: Shoot!");
            setState(STATE_SHOOT);
        }
    } else if (state == STATE_SHOOT) {
        // This state raise the correct kick flag for some time
        walk->control(false);

        if (t > 0.25 && !walk->isKicking()) {
            setState(STATE_NEAR);
        }
    }

    bind->push();
}

void Approach::enterState(std::string state)
{
    if (state == STATE_STOPPING) {
        walk->control(false);
        return;
    }

    t = 0;

    if (state == STATE_SHOOT) {
        std::string kickName;
        if (expectedKick == "lateral") {
            kickRight = (getKickCap().getSignedValue() > 0);
        } else {
            kickRight = (ballY < 0);
        }
        requestKick();
    }
}

void Approach::exitState(std::string state)
{
    if (state == STATE_STOPPING) {
        return;
    }

    if (state == STATE_SHOOT) {
        nbShoot++;
        elapsedLastShoot = 0;
    }
}


/**
 * REMOVED:
 * - Extra pitch time (rise head when shoot)
 * - Static shoot
 * - Scanning for ballls and goals
 */
