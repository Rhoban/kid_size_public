#include <functional>
#include <algorithm>
#include <rhoban_geometry/segment.h>
#include <rhoban_geometry/circle.h>
#include <services/LocalisationService.h>
#include <robocup_referee/constants.h>
#include <rhoban_geometry/circle.h>
#include "rhoban_utils/logging/logger.h"
#include "MCKickController.h"
#include "services/StrategyService.h"

using namespace rhoban_geometry;
using namespace rhoban_utils;
using namespace robocup_referee;

static rhoban_utils::Logger logger("MCKickController");

MCKickController::MCKickController()
{
  Move::initializeBinding();

  // Strategy file to load
  bind->bindNew("strategyFile", strategyFile)
      ->defaultValue("kickStrategy_with_grass.json")
      ->comment("Strategy file")
      ->persisted(true);

  // Should we avoid opponents
  bind->bindNew("avoidOpponents", avoidOpponents, RhIO::Bind::PullOnly)
      ->defaultValue(true)
      ->comment("Avoid the opponents?");

  // Reload strategy file
  bind->bindFunc("reloadStrategy", "Reload the strategy file", &MCKickController::cmdReloadStrategy, *this);

  bind->pull();

  // Loading strategy
  if (!strategy.fromJson(strategyFile))
  {
    logger.error("Can't load kick strategy file %s", strategyFile.c_str());
  }

  // Load available kicks
  kmc.loadFile();

  available = false;
  shouldReload = true;
  thread = new std::thread([this] { this->execute(); });
}

void MCKickController::execute()
{
  double grassOffset = 0;
  StrategyService* strategyService;

  // Waiting for scheduler to be referenced
  while (getScheduler() == nullptr)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  strategyService = getServices()->strategy;

  while (true)
  {
    // We should reload the scores for the kick strategy
    // This process re-generates the kick templates and model and can take ~10s to complete
    if (shouldReload || strategyService->getGrassOffset() != grassOffset)
    {
      available = false;
      logger.log("Reloading kickValueIteration (%d, %f, %f)", shouldReload, strategyService->getGrassOffset(),
                 grassOffset);
      shouldReload = false;
      grassOffset = strategyService->getGrassOffset();
      kickValueIteration = KickValueIteration("", 0.2, 5, 0, 10, grassOffset);
      kickValueIteration.loadScores(strategy);
      available = true;
    }

    // Updating the kicking strategy if the move is running
    if (isRunning())
    {
      mutex.lock();
      auto ballField = _ballField;
      auto teamMatesField = _teamMatesField;
      auto opponentsField = _opponentsField;
      auto opponentsRadius = _opponentsRadius;
      auto ball = _ballField;
      auto robotPos = _robotField;
      mutex.unlock();

      auto rewardFunc = [this, &ball, &robotPos, &teamMatesField, &opponentsField,
                         &opponentsRadius](Point fromPos, Point toPos, bool success) -> double {
        double penalty = 0;

        // We can't kick a ball through the opponents
        if (avoidOpponents)
        {
          Segment kick(fromPos, toPos);
          for (auto& opponent : opponentsField)
          {
            Point ballToOpponent = opponent - fromPos;
            if (ballToOpponent.getLength() < opponentsRadius * 2)
            {
              ballToOpponent.normalize(opponentsRadius * 2);
            }
            Circle opponentCircle(fromPos + ballToOpponent, opponentsRadius);

            if (kick.intersects(opponentCircle))
            {
              if (success)
              {
                penalty = -30;
              }
              else
              {
                penalty = -60;
              }
              break;
            }
          }
        }

        // We suppose that allies will be able to walk 0.5m more than us before we kick the ball
        double walkDistance = (robotPos - ball).getLength() + 0.5;

        // Changing fromPos to be the closer ally to the ball after kick instead of the ball
        // before kick (i.e the robot that is kicking)
        for (auto& entry : teamMatesField)
        {
          Point matePos(entry.second.x(), entry.second.y());
          Point matePosToBall = toPos - matePos;
          if (matePosToBall.getLength() < walkDistance)
          {
            matePos = toPos;
          }
          else
          {
            matePos = matePos + matePosToBall.normalize(walkDistance);
          }

          if ((matePos - toPos).getLength() < (fromPos - toPos).getLength())
          {
            fromPos = matePos;
          }
        }

        return kickValueIteration.travelReward(fromPos, toPos, success) + penalty;
      };

      auto action = kickValueIteration.bestAction(kickValueIteration.stateForFieldPos(ball.x, ball.y), rewardFunc);

      mutex.lock();
      _bestAction = action;
      mutex.unlock();
    }

    // Sleeping 10ms
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

std::string MCKickController::cmdReloadStrategy()
{
  bind->pull();
  std::stringstream ss;

  if (!strategy.fromJson(strategyFile))
  {
    ss << "Can't load strategy file " << strategyFile;
  }
  else
  {
    ss << "Strategy " << strategyFile << " loaded.";
  }

  shouldReload = true;
  return ss.str();
}

std::string MCKickController::getName()
{
  return "mc_kick_controler";
}

void MCKickController::onStart()
{
  bind->pull();
  forceUpdate = true;
}

void MCKickController::onStop()
{
}

void MCKickController::step(float elapsed)
{
  bind->pull();

  mutex.lock();
  // Populating fields for the thread from the localisation service
  LocalisationService* localisation = getServices()->localisation;
  _ballField = localisation->getBallPosField();
  _robotField = localisation->getFieldPos();
  _teamMatesField = localisation->getTeamMatesField();
  _opponentsField = localisation->getOpponentsField();
  _opponentsRadius = localisation->opponentsRadius;

  if (!available)
  {
    _bestAction = strategy.actionFor(_ballField.x, _ballField.y);
  }

  // Collecting the best action produced by the thread
  if (_bestAction.kick != "")
  {
    tolerance = _bestAction.tolerance / 2.0;

    allowed_kicks.clear();
    if (_bestAction.kick == "opportunist")
    {
      allowed_kicks.push_back("classic");
      allowed_kicks.push_back("lateral");
    }
    else
    {
      allowed_kicks.push_back(_bestAction.kick);
    }
  }

  kick_dir = rad2deg(_bestAction.orientation);
  mutex.unlock();
}
