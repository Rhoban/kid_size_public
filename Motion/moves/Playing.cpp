#include <math.h>
#include <services/DecisionService.h>
#include <services/LocalisationService.h>
#include <services/RefereeService.h>
#include <services/TeamPlayService.h>
#include <services/StrategyService.h>
#include <services/CaptainService.h>
#include <rhoban_utils/control/control.h>
#include <robocup_referee/constants.h>
#include "Head.h"
#include "Placer.h"
#include "Playing.h"
#include "Walk.h"
#include "Kick.h"
#include "rhoban_utils/logging/logger.h"

#define STATE_APPROACH "approach"
#define STATE_WALKBALL "walkBall"
#define STATE_SEARCH "search"
#define STATE_LET_PLAY "letPlay"
#define STATE_BACKWARD "backward"
#define STATE_STOPPING "stopping"
#define STATE_LOCALIZE "localize"

static rhoban_utils::Logger logger("PlayingSTM");

using namespace hl_communication;
using namespace rhoban_geometry;
using namespace rhoban_utils;
using namespace rhoban_team_play;
using namespace robocup_referee;

PlayingMove::PlayingMove(Walk* walk, Kick* kick) : walk(walk), kick(kick)
{
  initializeBinding();

  bind->bindNew("state", STM::state, RhIO::Bind::PushOnly)->comment("State of the Playing STM");

  bind->bindNew("localizeWalkDuration", localizeWalkDuration, RhIO::Bind::PullOnly)
      ->defaultValue(10.0)
      ->comment("Walk forward during this duration");

  bind->bindNew("useKickController", useKickController, RhIO::Bind::PullOnly)
      ->defaultValue(true)
      ->comment("Use the kick controller");

  bind->bindNew("stopOnHandle", stopOnHandle, RhIO::Bind::PullOnly)
      ->defaultValue(false)
      ->comment("Stop playing when handled (debug handling)");

  bind->bindNew("teamConfidence", teamConfidence, RhIO::Bind::PullOnly)
      ->defaultValue(true)
      ->comment("Am I confident in my team?");

  bind->bindNew("walkBallDistance", walkBallDistance, RhIO::Bind::PullOnly)
      ->defaultValue(1.2)
      ->comment("Distance to run the approach to the ball [m]");

  bind->bindNew("avoidRadius", avoidRadius, RhIO::Bind::PullOnly)
      ->defaultValue(1.0)
      ->comment("Radius [m] to avoid colliding the ball while placing");
}

std::string PlayingMove::getName()
{
  return "playing";
}

void PlayingMove::onStart()
{
  head = (Head*)getMoves()->getMove("head");
  placer = (Placer*)getMoves()->getMove("placer");
  auto& decision = getServices()->decision;

  if (!decision->isThrowInRunning)
  {
    head->setDisabled(false);
  }

  setState(STATE_SEARCH);

  // Ensuring fieldQThreshold is reasonable
  if (RhIO::Root.getValueFloat("decision/fieldQThreshold").value > 0.9)
  {
    logger.warning("High value for decision/fieldQThreshold");
  }

  backwardT = 10;
}

void PlayingMove::onStop()
{
  setState(STATE_STOPPING);
}

void PlayingMove::step(float elapsed)
{
  bind->pull();

  auto decision = getServices()->decision;
  auto captain = getServices()->captain;
  StrategyOrder order = captain->getMyOrder();
  t += elapsed;

  if (decision->handled && stopOnHandle)
  {
    walk->control(false);
    stop();
  }

  if (state == STATE_LOCALIZE)
  {
    localizeStep(elapsed);
  }
  else if (state == STATE_APPROACH)
  {
    approachStep(elapsed);
  }
  else if (state == STATE_WALKBALL)
  {
    walkBallStep(elapsed);
  }
  else if (!decision->shouldLetPlay && order.action() == Action::GOING_TO_KICK)
  {
    logger.log("Captain ordered me to handle the ball, going to walkball");
    setState(STATE_WALKBALL);
  }

  // We are letting the other play
  if (state == STATE_LET_PLAY)
  {
    letPlayStep(elapsed);
  }
  else
  {
    if (decision->shouldLetPlay || order.action() == Action::POSITIONING)
    {
      // No confidence in the team, we should see the ball to enter let play,
      // else we are in search state
      if (decision->isBallQualityGood)
      {
        logger.log("Letting play");
        setState(STATE_LET_PLAY);
      }
      else if (teamConfidence && decision->ballIsShared)
      {
        // We have confidence in the team, even if the ball is shared we
        // still enter this state and avoid searching for it
        logger.log("Letting play (confident)");
        setState(STATE_LET_PLAY);
      }
    }
  }

  auto localisation = getServices()->localisation;
  auto ball = localisation->getBallPosField();
  if (decision->isBallQualityGood && ball.getLength() < 2)
  {
    backwardT = 0;
  }
  else
  {
    backwardT += elapsed;
  }

  if (state != STATE_SEARCH && state != STATE_BACKWARD)
  {
    if (order.action() == Action::SEARCHING_BALL && !decision->isBallQualityGood)
    {
      logger.log("Captain ordered me to search the ball and I don't find it");
      if (backwardT < 3)
      {
        logger.log("-> Going to backward");
        setState(STATE_BACKWARD);
      }
      else
      {
        logger.log("-> Going to search");
        setState(STATE_SEARCH);
      }
    }
  }

  // We are walking backward
  if (state == STATE_BACKWARD)
  {
    walk->control(true, -walk->maxStepBackward, 0, 0);

    if (t > 5.0)
    {
      logger.log("Spent 5 seconds going backward, now moving to search");
      setState(STATE_SEARCH);
    }
  }

  // When field quality is low go to state localize
  if (state != STATE_LOCALIZE && !decision->isFieldQualityGood)
  {
    logger.log("Requiring more field observations, going to localize");
    setState(STATE_LOCALIZE);
  }

  bind->push();
}

void PlayingMove::localizeStep(float elapsed)
{
  auto referee = getServices()->referee;
  auto decision = getServices()->decision;

  // Walk forward when the robot enter the field
  if (referee->getTimeSincePlaying() < localizeWalkDuration)
  {
    walk->control(true, walk->maxStep);
  }
  else
  {
    // Behavior depend on the time spent in the state
    // Robot repeat a simple loop:
    // - wait (Robot is scanning)
    // - rotate
    // - wait
    // - walk forward

    // All of this could be parameters
    double locWait = 4;       //[s]
    double locRotTime = 6;    //[s]
    double locWalkTime = 10;  //[s]
    double localizePeriod = 2 * locWait + locRotTime + locWalkTime;

    // Avoid moving at full speed there (less risky)
    double walkSpeed = 0.7 * walk->maxStep;
    double turnSpeed = 0.7 * walk->maxRotation;
    double affix = fmod(t, localizePeriod);

    if (affix < locWait)
    {
      walk->control(false);
    }
    else if (affix < locWait + locRotTime)
    {
      walk->control(true, 0, 0, turnSpeed);
    }
    else if (affix < 2 * locWait + locRotTime)
    {
      walk->control(false);
    }
    else
    {
      walk->control(true, walkSpeed);
    }
  }
  // Once quality is good enough, go to search state
  if (decision->isFieldQualityGood)
  {
    logger.log("I am now localized, going to search the ball");
    setState(STATE_SEARCH);
  }
}

void PlayingMove::approachStep(float elapsed)
{
  auto decision = getServices()->decision;
  auto loc = getServices()->localisation;
  auto ball = loc->getBallPosSelf();
  auto dist = ball.getLength();

  if (!kick->isRunning())  // We don't want to stop approach if there is an ongoing kick
  {
    if (!decision->isBallQualityGood)
    {
      logger.log("Ball has been lost while in 'approach', going to 'backward'");
      setState(STATE_BACKWARD);
    }
    else if (dist > walkBallDistance * 1.1)
    {
      logger.log("Ball is now too far (%f, %f), going to 'walkBall'", ball.x, ball.y);
      setState(STATE_WALKBALL);
    }
  }
}

void PlayingMove::walkBallStep(float elapsed)
{
  auto decision = getServices()->decision;
  auto loc = getServices()->localisation;
  auto captain = getServices()->captain;

  auto ball = loc->getBallPosSelf();
  auto dist = ball.getLength();

  if (dist < walkBallDistance && decision->isBallQualityGood)
  {
    logger.log("I am now in range of the ball (%f, %f), going to 'approach'", ball.x, ball.y);
    setState(STATE_APPROACH);
  }

  // Defend inflection
  Point ballField;
  if (decision->isBallQualityGood)
  {
    ballField = loc->getBallPosField();
  }
  else
  {
    if (!captain->importCommonBall(&ballField))
    {
      logger.error("No common ball found, while it should be present");
    }
  }
  auto goalField = loc->getOurGoalPosField();
  double c = -ballField.x * 2 / Constants::field.field_length;
  if (c < 0)
  {
    c = 0;
  }
  c *= dist;
  if (c > walkBallDistance * 0.85)
  {
    c = walkBallDistance * 0.85;
  }
  auto ballGoalField = (goalField - ballField).normalize(c);
  ballGoalField += ballField;

  placer->goTo(ballGoalField.x, ballGoalField.y, 0);
}

void PlayingMove::letPlayStep(float elapsed)
{
  auto decision = getServices()->decision;
  auto loc = getServices()->localisation;
  auto teamPlay = getServices()->teamPlay;
  auto captain = getServices()->captain;
  auto referee = getServices()->referee;
  StrategyOrder order = captain->getMyOrder();

  if (!teamConfidence || order.action() != Action::POSITIONING)
  {
    // Letting play: case 1, robot is not placing or there is no trust in team (typically, during the 10 seconds
    // buffer after game interruptions)

    auto ball = loc->getBallPosField();
    float letPlayRadius = decision->letPlayRadius;
    Point goalCenter(-Constants::field.field_length / 2, 0);
    Point target = ball + (goalCenter - ball).normalize(letPlayRadius);
    double ballAzimuth = (ball - goalCenter).getTheta().getSignedValue();

    std::vector<rhoban_geometry::Circle> obstacles;
    obstacles.push_back(rhoban_geometry::Circle(ball, letPlayRadius));

    placer->goTo(target.x, target.y, ballAzimuth, obstacles);
  }
  else
  {
    // Letting play: case 2, robot is placing and trusts its team
    // XXX: Some values should be Rhiozed here
    Point ball, ballTarget;
    ball = Point(decision->shareX, decision->shareY);
    ballTarget = Point(decision->ballTargetX, decision->ballTargetY);

    // Ball handler
    // XXX: The captain ball handler could be used instead
    std::unique_ptr<RobotMsg> handler_msg;
    for (const auto& entry : teamPlay->allInfo())
    {
      const RobotMsg& msg = entry.second;
      if (msg.has_intention() && msg.intention().has_action_planned())
      {
        Action action = msg.intention().action_planned();
        if (action == Action::GOING_TO_KICK || action == Action::KICKING)
        {
          handler_msg.reset(new RobotMsg(msg));
          break;
        }
      }
    }

    const PoseDistribution& target_pose = order.target_pose();

    // Avoiding the ball
    std::vector<rhoban_geometry::Circle> obstacles;
    obstacles.push_back(Circle(ball.x, ball.y, avoidRadius));

    if (referee->isOpponentKickOffStart())
    {
      obstacles.push_back(rhoban_geometry::Circle(Point(0, 0), decision->letPlayRadius));
    }

    // Avoiding the ball trajectory
    // Point tmp = ball;
    // Point kickVect = (ballTarget - ball);
    // double norm = kickVect.getLength();
    // int parts = norm/0.3;
    // if (parts > 5) parts = 5;
    // for (int k=1; k<=parts; k++) {
    //     Point pos = ball + kickVect.normalize(norm*k/(float)parts);
    //     obstacles.push_back(Circle(pos.x, pos.y, 0.5));
    // }

    if (handler_msg)
    {
      if (handler_msg->has_perception())
      {
        const PositionDistribution& handler_pos = handler_msg->perception().self_in_field(0).pose().position();
        obstacles.push_back(Circle(handler_pos.x(), handler_pos.y(), avoidRadius));
      }
    }
    placer->goTo(target_pose.position().x(), target_pose.position().y(), target_pose.dir().mean(), obstacles);
  }

  if (!teamConfidence && !decision->isBallQualityGood)
  {
    logger.log("I don't see the ball, going to search");
    setState(STATE_SEARCH);
  }
}

void PlayingMove::enterState(std::string state)
{
  logger.log("Entering state %s", state.c_str());
  auto& strategy = getServices()->strategy;

  if (state == STATE_LOCALIZE)
  {
    head->setForceLocalize(true);
  }
  else
  {
    head->setForceLocalize(false);
  }

  if (state == STATE_APPROACH)
  {
    startMove(strategy->getDefaultApproach(), 0.0);
  }

  if (state == STATE_APPROACH || state == STATE_WALKBALL || state == STATE_LET_PLAY)
  {
    if (useKickController)
    {
      startMove(strategy->getDefaultKickController(), 0.0);
    }
  }

  if (state == STATE_SEARCH)
  {
    startMove("search", 0.0);
  }

  if (state == STATE_WALKBALL || state == STATE_LET_PLAY)
  {
    startMove("placer", 0.0);
  }

  t = 0;
}

void PlayingMove::exitState(std::string state)
{
  logger.log("Exiting state %s", state.c_str());
  auto& strategy = getServices()->strategy;

  if (state == STATE_APPROACH)
  {
    stopMove(strategy->getDefaultApproach(), 0.0);
  }

  if (state == STATE_APPROACH || state == STATE_WALKBALL || state == STATE_LET_PLAY)
  {
    stopMove(strategy->getDefaultKickController(), 0.0);
  }

  if (state == STATE_BACKWARD || state == STATE_LOCALIZE || state == STATE_LET_PLAY)
  {
    walk->control(false);
  }

  if (state == STATE_WALKBALL || state == STATE_LET_PLAY)
  {
    stopMove("placer", 0.0);
  }

  if (state == STATE_SEARCH)
  {
    stopMove("search", 0.0);
  }

  // Avoid glitching off the walk during transition between placing and approach
  if ((state == STATE_WALKBALL && nextState == STATE_APPROACH) ||
      (state == STATE_APPROACH && nextState == STATE_WALKBALL))
  {
    walk->control(true);
  }
}
