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

// Uncomment this to force using right foot with classic kick (for debugging purpose)
// #define DEBUG_FORCE_KICK_RIGHT_CLASSIC

static rhoban_utils::Logger logger("ApproachPotential" /*, LoggerDebug*/);

using namespace rhoban_geometry;
using namespace rhoban_utils;

#define STATE_STOPPING "stopping"
#define STATE_PLACE "place"
#define STATE_STEPPING "stepping"
#define STATE_SHOOT "shoot"

ApproachPotential::Target::Target()
{
}

ApproachPotential::Target::Target(Point position, Angle yaw, bool rightKick, std::string kickName, double tolerance)
  : position(position), yaw(yaw), rightKick(rightKick), kickName(kickName), tolerance(tolerance)
{
}

ApproachPotential::ApproachPotential(Walk* walk) : ApproachMove(walk), currentTolerance(0)
{
  Move::initializeBinding();
  initBindings();

  // State
  bind->bindNew("state", STM::state, RhIO::Bind::PushOnly)->comment("Approach STM state");
  bind->bindNew("repulsion", repulsion, RhIO::Bind::PullOnly)->defaultValue(0.5);

  // Servoing
  bind->bindNew("stepP", stepP, RhIO::Bind::PullOnly)->defaultValue(6);
  bind->bindNew("lateralP", lateralP, RhIO::Bind::PullOnly)->defaultValue(6);
  bind->bindNew("stepI", stepI, RhIO::Bind::PullOnly)->defaultValue(0.0);
  bind->bindNew("lateralI", lateralI, RhIO::Bind::PullOnly)->defaultValue(0.0);
  bind->bindNew("stepPunch", stepPunch, RhIO::Bind::PullOnly)->defaultValue(0);
  bind->bindNew("rotationP", aligner.k_p, RhIO::Bind::PullOnly)->defaultValue(1.2);
  bind->bindNew("rotationI", aligner.k_i, RhIO::Bind::PullOnly)->defaultValue(0.0);

  bind->bindNew("placementDistance", placementDistance, RhIO::Bind::PullOnly)->defaultValue(0.35);

  // Don't walk
  bind->bindNew("dontWalk", dontWalk, RhIO::Bind::PullOnly)->defaultValue(false);

  bind->bindNew("ballX", ballX, RhIO::Bind::PushOnly);
  bind->bindNew("ballY", ballY, RhIO::Bind::PushOnly);
}

Angle ApproachPotential::getKickCap()
{
  return ApproachMove::getKickCap() + Angle(currentTolerance);
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

void ApproachPotential::getControl(const Target& target, const Point& ball, double& x, double& y, double& yaw)
{
  double dist = target.position.getLength();

  // x = target.position.x*100;
  // y = target.position.y*100;
  // yaw = target.yaw.getSignedValue();
  // return;

  // Going directly (to debug the target)
  // logger.log("Target: %f, %f, %f\n", target.position.x, target.position.y, target.yaw.getSignedValue());
  // walk->control(true, target.position.x*100, target.position.y*100, target.yaw.getSignedValue());

  // Magical potential field
  // See Motion/python/potential.py
  double X(0), Y(0);
  double norm2 = sqrt(pow(target.position.x, 2) + pow(target.position.y, 2));
  if (norm2 > 0)
  {
    X = ball.x * repulsion * pow(pow(ball.x, 2) + pow(ball.y, 2), -1 / 2 * repulsion) *
            sqrt(pow(target.position.x, 2) + pow(target.position.y, 2)) / (pow(ball.x, 2) + pow(ball.y, 2)) -
        target.position.x * pow(pow(ball.x, 2) + pow(ball.y, 2), -1 / 2 * repulsion) /
            sqrt(pow(target.position.x, 2) + pow(target.position.y, 2));
    Y = ball.y * repulsion * pow(pow(ball.x, 2) + pow(ball.y, 2), -1 / 2 * repulsion) *
            sqrt(pow(target.position.x, 2) + pow(target.position.y, 2)) / (pow(ball.x, 2) + pow(ball.y, 2)) -
        target.position.y * pow(pow(ball.x, 2) + pow(ball.y, 2), -1 / 2 * repulsion) /
            sqrt(pow(target.position.x, 2) + pow(target.position.y, 2));
  }

  // XXX: Some below variable should be rhiorized
  Point control(-X, -Y);

  // XXX: dist here may be replaced with the distance following the potential fields
  double P = dist * 100;
  control.normalize(P);

  // Applying punch
  if (dist < 0.65)
  {
    if (control.x > 2)
      control.x += stepPunch;
    if (control.x < -2)
      control.x -= stepPunch;
    if (control.y > 2)
      control.y += stepPunch;
    if (control.y < -2)
      control.y -= stepPunch;
  }

  // Normalizing using walk max speeds
  if (control.x > walk->maxStep)
    control.normalize(walk->maxStep);
  if (control.x < -walk->maxStepBackward)
    control.normalize(walk->maxStepBackward);
  if (control.y > walk->maxLateral)
    control.normalize(walk->maxLateral);

  double goalCap = atan2(control.y, control.x);
  double ballCap = atan2(ball.y, ball.x);
  double targetCap = deg2rad(target.yaw.getSignedValue());

  double error = goalCap;
  // if (dist < 0.65)
  // {
  //   // We are near the ball, let's face it
  //   // error = ballCap;
  // }
  if (dist < placementDistance)
  {
    // We are near the goal, let's face the goal
    error = targetCap;
  }

  if (dist > placementDistance)
  {
    // Avoiding walking backward when error azimuth is high
    control *= std::max<double>(0, cos(error));
  }

  // Response
  x = control.x;

  if (dist > placementDistance)
  {
    y = 0;
  }
  else
  {
    y = control.y;
  }
  yaw = rad2deg(error);
}

void ApproachPotential::step(float elapsed)
{
  bind->pull();

  stepper.k_p = stepP;
  lateraler.k_p = lateralP;

  stepper.k_i = stepI;
  lateraler.k_i = lateralI;

  auto loc = getServices()->localisation;

  if (state == STATE_SHOOT)
  {
    if (!walk->isKicking())
    {
      setState(STATE_PLACE);
    }
  }

  if (state == STATE_PLACE)
  {
    // Target yaw
    auto cap = ApproachMove::getKickCap();

    // Ball position
    auto ball = loc->getBallPosSelf();
    ball = walk->trunkToFlyingFoot(ball);
    ballX = ball.x;  // XXX: To debug
    ballY = ball.y;

    std::vector<Target> targets;
    lastFootChoice += elapsed;
    if (lastFootChoice > 3.0)
    {
      left = ball.y > 0;
      lastFootChoice = 0;
    }

    auto allowedKicks = getAllowedKicks();

#ifdef DEBUG_FORCE_KICK_RIGHT_CLASSIC
    left = false;
    allowedKicks.clear();
    allowedKicks.push_back("classic");
#endif

    if (allowedKicks.size())
    {
      for (const std::string& name : allowedKicks)
      {
        for (bool isKickRight : { false, true })
        {
          if (name == "classic" || name == "small")
          {
            if (isKickRight != !left)
              continue;
          }

          std::vector<double> toleranceAngles;
          toleranceAngles.push_back(0);
          double toleranceStep = 1;  // Previously 5
          double kick_tol_rad = kmc.getKickModel(name).getKickZone().getThetaTol();
          double maxAlpha = getKickTolerance() - rad2deg(kick_tol_rad);

#ifdef DEBUG_FORCE_KICK_RIGHT_CLASSIC
          maxAlpha = 0;
#endif

          for (double alpha = toleranceStep; alpha < maxAlpha; alpha += toleranceStep)
          {
            toleranceAngles.push_back(-alpha);
            toleranceAngles.push_back(alpha);
          }

          for (const double& tolerance : toleranceAngles)
          {
            // Importing wished pos from kick_model
            const csa_mdp::KickZone& kick_zone = kmc.getKickModel(name).getKickZone();
            Eigen::Vector3d wishedPos = kick_zone.getWishedPos(isKickRight);
            // Computing target
            double targetX = wishedPos(0);
            double targetY = wishedPos(1);
            Angle lateralAngle(rad2deg(wishedPos(2)));
            Angle thetaWished = cap + lateralAngle + Angle(tolerance);

            Point kickRelativePos(targetX, targetY);
            Point offset = kickRelativePos.rotation(thetaWished);

            targets.push_back(Target(ball - offset, thetaWished, isKickRight, name, tolerance));
          }
        }
      }

      Target target;
      double bestScore = -1;
      auto ballField = loc->getBallPosField();
      Angle defendTargetDir = Angle(180) - loc->getOurBallToGoalDirSelf();
      // std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;
      // std::cout << "OUR GOAL: " << defendTargetDir << std::endl;

      for (auto t : targets)
      {
        double cX, cY, cYaw;
        getControl(t, ball, cX, cY, cYaw);

        // Score is a rough time estimation, we suppose that we will walk at
        // max speed
        double score = (t.position.getLength() * 1000.0) / walk->maxStep;

        // That we have to align with current yaw (that can be potential field provided)
        double degsToTravel = fabs(cYaw);

        // And then have to align with target yaw
        degsToTravel += fabs((t.yaw - cYaw).getSignedValue());

        // Again, we suppose that we rotate at max speed
        score += degsToTravel / walk->maxRotation;

        if (ballField.x < 0)
        {
          auto defendError = fabs((t.yaw - defendTargetDir).getSignedValue());
          if (defendError > 30)
          {
            score *= 30 * defendError;
          }
        }
        // std::cout << "Score for [n: " << t.position.getLength() << ", x: " << t.position.x << ",y: " << t.position.y
        //           << ",t: " << cYaw << " (travel°: " << degsToTravel << ")] " << t.kickName << " / "
        //           << t.yaw.getSignedValue() << " : " << score << std::endl;

        if (bestScore < 0 || score < bestScore)
        {
          target = t;
          bestScore = score;
        }
      }

      // std::cout << "Target: " << target.position.x << ", " << target.position.y << ", " << target.yaw.getSignedValue()
      //           << " (kick: " << target.kickName << ")" << std::endl;

      // Setting expectedKick
      expectedKick = target.kickName;
      kickRight = target.rightKick;
      currentTolerance = target.tolerance;

      updateKickScore(elapsed);

      if (kick_score >= 1.0)
      {
        walk->control(false, 0, 0, 0);
        requestKick();
        setState(STATE_SHOOT);
      }
      else
      {
        // Applying control
        double controlX, controlY, controlYaw;
        getControl(target, ball, controlX, controlY, controlYaw);

        controlX = stepper.update(controlX, elapsed);
        controlY = lateraler.update(controlY, elapsed);
        controlYaw = aligner.update(controlYaw, elapsed);

        walk->control(!dontWalk, controlX, controlY, controlYaw);
      }
    }
    else
    {
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
