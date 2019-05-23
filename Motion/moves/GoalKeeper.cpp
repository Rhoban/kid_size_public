#include "GoalKeeper.h"
#include <math.h>

#include "Placer.h"
#include "Walk.h"
#include <services/LocalisationService.h>
#include <services/StrategyService.h>
#include <services/DecisionService.h>
#include <robocup_referee/constants.h>
#include "Playing.h"
#include "rhoban_utils/logging/logger.h"

static rhoban_utils::Logger logger("GoalKeeper");

using namespace robocup_referee;
using namespace rhoban_utils;
using namespace rhoban_geometry;

#define STATE_INIT "init"        // init of the goal, begining of the match
#define STATE_WAIT "wait"        // wait for the ball in danger zone (home position)
#define STATE_ALIGNBALL "align"  // align goalkeeper with ball in x position
#define STATE_ATTACK "attack"    // go to ball and shoot it out
#define STATE_STOP "stop"        // stop and take the block position
#define STATE_GOHOME "gohome"    // go back to home position (align then home)

/*
 *                         y
 *  |----------------------------------------------|
 *  |____           |      |                       |
 *  |___|           |      |                       |
 *  |  ||           |      |                       |
 *  |  ||           |      |                       |
 *  |  ||           |      0--->x                  |
 *  | a||           |      |                       |
 *  |  ||       c   |      |                       |
 *  |__||           |      |                       |
 *  |__b|           |      |                       |
 *  |               |      |                       |
 *  |----------------------------------------------|
 *
 * a = attack
 * b = attacklimit
 * c = dangerzone
 */

GoalKeeper::GoalKeeper(Walk* walk, Placer* placer) : walk(walk), placer(placer)
{
  initializeBinding();

  bind->bindNew("state", STM::state, RhIO::Bind::PushOnly)->comment("State of the GoalKeeper STM");

  bind->bindNew("homeX", homeX, RhIO::Bind::PullOnly)->comment("in front of cages")->defaultValue(-4.0);
  bind->bindNew("homeY", homeY, RhIO::Bind::PullOnly)->comment("by default middle of field")->defaultValue(0.0);

  bind->bindNew("xAttack", xAttack, RhIO::Bind::PullOnly)
      ->comment("Distance x between goal line and limit to attack ")
      ->defaultValue(1.5);
  bind->bindNew("yAttack", yAttack, RhIO::Bind::PullOnly)
      ->comment("Distance y between home and limit to attack")
      ->defaultValue(1.5);
  bind->bindNew("distanceAttack", distanceAttack, RhIO::Bind::PullOnly)
      ->comment("Distance x between robot and limit to attack")
      ->defaultValue(0.5);

  bind->bindNew("xAttackHys", xAttackHys, RhIO::Bind::PullOnly)
      ->comment("Distance x between goal line and limit to attack hysteresis")
      ->defaultValue(1.75);
  bind->bindNew("yAttackHys", yAttackHys, RhIO::Bind::PullOnly)
      ->comment("Distance y between home and limit to attack hysteresis")
      ->defaultValue(1.75);

  bind->bindNew("xIgnoreBall", xIgnoreBall, RhIO::Bind::PullOnly)
      ->comment("Distance x between goal line and limit of the danger zone")
      ->defaultValue(3.5);

  bind->bindNew("xApprox", xApprox, RhIO::Bind::PullOnly)
      ->comment("Acceptable distance x between good position and goal position")
      ->defaultValue(0.1);
  bind->bindNew("yApprox", yApprox, RhIO::Bind::PullOnly)
      ->comment("Acceptable distance y between good position and goal position")
      ->defaultValue(0.1);

  bind->bindNew("t", t, RhIO::Bind::PushOnly)->comment("Duration of the current state");
}

std::string GoalKeeper::getName()
{
  return "goal_keeper";
}

void GoalKeeper::onStart()
{
  bind->pull();
  setState(STATE_INIT);
  RhIO::Root.setFloat("/moves/placer/marginX", 0.1);
  RhIO::Root.setFloat("/moves/placer/marginY", 0.1);
  RhIO::Root.setFloat("/moves/placer/marginAzimuth", 10);
}

void GoalKeeper::onStop()
{
  setState(STATE_WAIT);
}
Point GoalKeeper::home()
{
  return Point(homeX, homeY);
}

// is the ball in a zone
bool GoalKeeper::ballInZone(float xd, float yd)
{
  auto loc = getServices()->localisation;
  auto ball = loc->getBallPosField();
  auto decision = getServices()->decision;

  return decision->isBallQualityGood && ball.x < (loc->getOurGoalPosField().x + xd) && fabs(ball.y) < yd;
}

// is the ball in the danger zone
bool GoalKeeper::ballInDangerZone()
{
  return ballInZone(xIgnoreBall, Constants::field.field_width + 1 / 2.0);
}

// is the ball in the attack hys zone
bool GoalKeeper::ballInAttackHysZone()
{
  return ballInZone(xAttackHys, yAttackHys);
}

// is the ball in attack zone
bool GoalKeeper::ballInAttackZone()
{
  return ballInZone(xAttack, yAttack);
}

// Are we safe ?  (ball too far from us
bool GoalKeeper::isBallSafe()
{
  return !ballInAttackZone() && !ballInAttackHysZone() && !ballInDangerZone();
}

// if ball in Danger Zone
Point GoalKeeper::alignBallPos()
{
  auto loc = getServices()->localisation;
  auto ball = loc->getBallPosField();
  ySign = 1;
  if (ball.y < 0.0)
    ySign = -1;

  float xGoal = loc->getOurGoalPosField().x - Constants::field.goal_depth;
  coeffa = (homeY - fabs(ball.y)) / (xGoal - ball.x);
  coeffb = homeY - coeffa * xGoal;

  yPos = std::min(homeX * coeffa + coeffb, (float)(Constants::field.goal_width / 2.0));

  if (ball.x < loc->getOurGoalPosField().x)
  {
    // 0.1 to avoid the goal pole
    return Point(loc->getOurGoalPosField().x - 0.1, ySign * yPos);
  }
  // if ball is near

  if (ball.x < homeX)
  {
    return Point(ball.x, ySign * yPos);
  }
  else
  {
    return Point(homeX, ySign * yPos);
  }
}

float GoalKeeper::getAngle()
{
  auto loc = getServices()->localisation;
  auto ball = loc->getBallPosField();
  auto pos = loc->getFieldPos();

  distX = fabs(pos.x - ball.x);
  if (ball.y > 0)
    distY = fabs(pos.y - ball.y);
  else
    distY = ball.y - pos.y;

  return Angle::fromXY(distX, distY).getSignedValue();
}

bool GoalKeeper::goodEnoughPos(Point pos, Point needed_pos)
{
  return (fabs(pos.x - needed_pos.x) < xApprox) && (fabs(pos.y - needed_pos.y) < yApprox);
}

void GoalKeeper::step(float elapsed)
{
  bind->pull();
  t += elapsed;

  if (state == STATE_INIT)
  {
    if (t < 1.0)
      return;
    setState(STATE_GOHOME);
    logger.log("state : starting match : GO_HOME");
    return;
  }

  auto loc = getServices()->localisation;
  auto decision = getServices()->decision;
  auto pos = loc->getFieldPos();
  auto ball = loc->getBallPosField();

  if (pos.getDist(ball) < distanceAttack)
  {
    if (decision->isBallQualityGood && state != STATE_ATTACK)
    {
      setState(STATE_ATTACK);
      logger.log("state : ball is near, ATTACK !");
    }
  }
  else
  {
    if (state == STATE_ATTACK)
    {
      if ((!decision->isBallQualityGood || !ballInAttackHysZone()))
      {
        setState(STATE_ALIGNBALL);
        logger.log("state : ball is not a danger anymore : ALIGN");
      }
    }

    else if (isBallSafe())
    {
      if (state == STATE_GOHOME)
      {
        if (placer->arrived)
        {
          setState(STATE_WAIT);
          logger.log("state : waiting for ball in dangerZone : WAIT");
        }
      }
      else
      {
        if (state != STATE_WAIT)
        {
          setState(STATE_GOHOME);
          logger.log("state : ball is far, going back home : GO_HOME");
        }
        else  // if STATE_WAIT
        {
          if (loc->fieldQ < 0.5)  // look around for better location
          {
            placer->goTo(homeX, homeY, 90);
          }
          else
          {
            placer->goTo(homeX, homeY, 0);
          }
        }
      }
    }

    else if (ballInAttackZone())
    {
      if (decision->isBallQualityGood && state != STATE_ATTACK && ball.y > loc->getOurGoalPosField().x)
      {
        setState(STATE_ATTACK);
        logger.log("state : ball in Attack Zone : ATTACK !");
      }
    }

    else if (state == STATE_STOP)
    {
      needed_pos = alignBallPos();

      if (pos.getDist(needed_pos) > 0.15)
      {
        setState(STATE_ALIGNBALL);
        placer->goTo(needed_pos.x, needed_pos.y, getAngle());
        logger.log("state : we are too far from optimized pos : ALIGN");
      }
      else if (fabs(rad2deg(normalizeRad(loc->getFieldOrientation())) - getAngle()) > 10)
      {
        setState(STATE_ALIGNBALL);
        placer->goTo(pos.x, pos.y, getAngle());
        logger.log("state : we are too far from optimized angle : ALIGN");
      }
    }

    else if (state == STATE_ALIGNBALL)
    {
      if (placer->arrived)
      {
        setState(STATE_STOP);
        logger.log("state : we are in position : STOP");
      }
      else
      {
        needed_pos = alignBallPos();
        placer->goTo(needed_pos.x, needed_pos.y, getAngle());
      }
    }

    else
    {
      needed_pos = alignBallPos();

      if (pos.getDist(needed_pos) > 0.1)
      {
        setState(STATE_ALIGNBALL);
        placer->goTo(needed_pos.x, needed_pos.y, getAngle());
        logger.log("state : ball in DangerZone, positionning : ALIGN");
      }
    }
  }

  bind->push();
}

void GoalKeeper::enterState(std::string state)
{
  bind->pull();
  auto& strategy = getServices()->strategy;
  t = 0.0;
  auto loc = getServices()->localisation;
  auto pos = loc->getFieldPos();

  if (state == STATE_GOHOME)
  {
    placer->goTo(homeX, homeY, 0);
    if (placedByHand)
    {
      placer->setDirectMode(false);
    }
    else
    {
      placer->setDirectMode(true);
    }
    auto loc = getServices()->localisation;
    auto pos = loc->getFieldPos();

    if (goodEnoughPos(pos, home()))
    {
      setState(STATE_WAIT);
    }

    else
    {
      startMove("placer", 0.0);
    }
  }
  else if (state == STATE_ATTACK)
  {
    startMove(strategy->getDefaultApproach(), 0.0);
    startMove("clearing_kick_controler", 0.0);
  }
  else if (state == STATE_ALIGNBALL)
  {
    needed_pos = alignBallPos();
    if (pos.getDist(needed_pos) < 0.5)
      placer->setDirectMode(false);
    else
      placer->setDirectMode(true);
    startMove("placer", 0.0);
  }
}

void GoalKeeper::exitState(std::string state)
{
  auto& strategy = getServices()->strategy;

  if (state == STATE_INIT)
  {
    auto loc = getServices()->localisation;
    auto pos = loc->getFieldPos();

    if (goodEnoughPos(pos, home()))
    {
      placedByHand = true;
    }
    else
    {
      placedByHand = false;
    }
  }
  else if (state == STATE_GOHOME)
  {
    stopMove("placer", 0.0);
    placer->setDirectMode(true);
  }
  else if (state == STATE_ALIGNBALL)
  {
    stopMove("placer", 0.0);
    placer->setDirectMode(true);
  }
  else if (state == STATE_ATTACK)
  {
    stopMove(strategy->getDefaultApproach(), 0.0);
    stopMove("clearing_kick_controler", 0.0);
  }
}
