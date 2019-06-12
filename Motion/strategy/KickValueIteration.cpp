#include <set>
#include <random>
#include <iostream>
#include <cmath>
#include "KickValueIteration.hpp"
#include <robocup_referee/constants.h>
#include <rhoban_geometry/segment.h>
#include <rhoban_geometry/circle.h>

using namespace robocup_referee;
using namespace rhoban_geometry;

KickValueIteration::KickValueIteration(std::string kicksFile, double accuracy, double angleAccuracy, bool dump,
                                       double tolerance, double grassOffset)
  : accuracy(accuracy), angleAccuracy(angleAccuracy), dump(dump), tolerance(tolerance)
{
  if (kicksFile == "")
  {
    kicks.loadFile();
  }
  else
  {
    kicks.loadFile(kicksFile);
  }
  kicks.setGrassConeOffset(grassOffset);
}

double KickValueIteration::travelRewardDefault(rhoban_geometry::Point fromPos, rhoban_geometry::Point toPos,
                                               bool success)
{
  if (success)
  {
    return 0;
  }

  return -(5 + (fromPos - toPos).getLength() / 0.15);
}

double KickValueIteration::rewardFor(State* from, State* state, double kickLength, travelRewardFunc travelFunc)
{
  if (state == &failState)
  {
    return -200;
  }

  // Source box gives the starting point on the field (where the ball is kicked)
  Point fromPos =
      Point(from->fieldX - Constants::field.field_length / 2.0, from->fieldY - Constants::field.field_width / 2.0);

  Point toPos;
  if (state->isSuccess)
  {
    // Estimating the kick arrival after a goal
    // XXX: We don't know kickLength here, we use an exagerated 10m kick since the goal is mainly to have a
    // good orientation and allow the final user of travelFunc to check if its kick intersects an opponetn for eg.
    Point kick(cos(state->lastKickOrientation) * kickLength, sin(state->lastKickOrientation) * kickLength);
    toPos = fromPos + kick;

    // std::cout << kickLength << std::endl;
    // std::cout << state->lastKickOrientation << std::endl;
    // std::cout << fromPos << std::endl;
    // std::cout << toPos << std::endl;
    // exit(0);
  }
  else
  {
    // Using the target box to estimate target position on floor
    toPos =
        Point(state->fieldX - Constants::field.field_length / 2.0, state->fieldY - Constants::field.field_width / 2.0);
  }

  if (travelFunc)
  {
    return travelFunc(fromPos, toPos, state->isSuccess);
  }
  else
  {
    return travelRewardDefault(fromPos, toPos, state->isSuccess);
  }
}

KickStrategy KickValueIteration::generate()
{
  KickStrategy strategy(accuracy);

  // Generate the states
  generateStates();

  // Genrate kick templates
  generateTemplate();

  // Generate the model of actions for each state
  generateModels();

  // Iterate over all the possibilities
  while (iterate())
    ;

  // Write the best actions in given strategy
  populateStrategy(strategy);

  return strategy;
}

void KickValueIteration::populateStrategy(KickStrategy& strategy)
{
  for (int y = 0; y < ySteps; y++)
  {
    for (int x = 0; x < xSteps; x++)
    {
      double X = accuracy * x;
      double Y = accuracy * y;
      KickValueIteration::State* state = &states[x][y];

      strategy.setAction(X, Y, bestAction(state));
    }
  }
}

KickStrategy::Action KickValueIteration::bestAction(KickValueIteration::State* state, travelRewardFunc travelFunc)
{
  Action bestAction;
  double bestScore = -1000;
  std::set<int> possibleOrientations;
  std::set<std::string> possibleKicks;

  // Collecting action scores
  std::map<Action, double> actionScores;
  for (auto& entry : state->models)
  {
    double actionScore = 0;
    for (auto& possibility : entry.second)
    {
      actionScore +=
          possibility.first *
          (rewardFor(state, possibility.second, kickLengths[entry.first.kick], travelFunc) + possibility.second->score);
    }

    // Storing action score
    actionScores[entry.first] = actionScore;

    if (actionScore > bestScore)
    {
      bestAction = entry.first;
      bestScore = actionScore;
    }
  }

  for (auto& entry : state->models)
  {
    auto tmpAction = entry.first;
    double actionScore = actionScores[tmpAction];

    // If the action is withing our time tolerance
    if (fabs(actionScore - bestScore) < tolerance)
    {
      possibleKicks.insert(tmpAction.kick);
      possibleOrientations.insert(round(tmpAction.orientation * 1000));

      if (dump)
      {
        // Debug draw of the kick arrows
        double R = 0.1;
        double kickX = state->fieldX + cos(tmpAction.orientation) * R;
        double kickY = state->fieldY + sin(tmpAction.orientation) * R;

        std::cout << state->fieldX << " " << state->fieldY << " " << actionScore << std::endl;
        std::cout << kickX << " " << kickY << " " << actionScore << std::endl;
        std::cout << std::endl << std::endl;
      }
    }
  }

  KickStrategy::Action resultAction;

  // XXX: We suppose that the possible orientations are dispatched evenly
  // resultAction.tolerance = std::max<double>(0, (possibleOrientations.size() - 1) * angleAccuracy * M_PI / 180.0);
  resultAction.tolerance = 0;

  resultAction.orientation = bestAction.orientation;
  resultAction.score = bestScore;

  if (possibleKicks.count("classic") && possibleKicks.count("lateral"))
  {
    resultAction.kick = "opportunist";
  }
  else
  {
    resultAction.kick = bestAction.kick;
  }

  return resultAction;
}

KickValueIteration::State::State() : isSuccess(false)
{
}

void KickValueIteration::generateStates()
{
  for (auto& kickName : getKickNames())
  {
    auto& kickModel = kicks.getKickModel(kickName);
    Eigen::Vector2d kickTarget = kickModel.applyKick(Eigen::Vector2d(0, 0), 0);

    kickLengths[kickName] = kickTarget.norm();
  }

  xSteps = 1 + Constants::field.field_length / accuracy;
  ySteps = 1 + Constants::field.field_width / accuracy;
  aSteps = 360 / angleAccuracy;

  successStates.clear();
  for (int a = 0; a < aSteps; a++)
  {
    State state;
    state.score = 0;
    state.lastKickOrientation = a * angleAccuracy * M_PI / 180.0;
    state.isSuccess = true;
    successStates.push_back(state);
  }
  failState.score = 0;

  for (int x = 0; x < xSteps; x++)
  {
    for (int y = 0; y < ySteps; y++)
    {
      State state;
      state.x = x;
      state.y = y;
      state.fieldX = x * accuracy;
      state.fieldY = y * accuracy;
      state.score = -1000;
      states[x][y] = state;
    }
  }
}

void KickValueIteration::generateTemplate()
{
  std::default_random_engine generator;
  // std::normal_distribution<double> posNoise(0, accuracy);

#define SAMPLES 10000

  for (int a = 0; a < aSteps; a++)
  {
    for (auto& kickName : getKickNames())
    {
      auto& kickModel = kicks.getKickModel(kickName);
      Action action;
      action.kick = kickName;
      action.orientation = (a * 2 * M_PI) / aSteps;

      std::map<std::pair<int, int>, int> count;

      for (size_t k = 0; k < SAMPLES; k++)
      {
        auto result = kickModel.applyKick(Eigen::Vector2d(0, 0), action.orientation, &generator);
        auto sample = Point(result[0], result[1]);

        int dX = round((sample.x) / accuracy);
        int dY = round((sample.y) / accuracy);

        count[std::pair<int, int>(dX, dY)] += 1;
      }

      for (auto& entry : count)
      {
        kickTemplate[action].push_back(PossibilityTemplate(entry.second / (double)SAMPLES, entry.first));
      }
    }
  }
}

void KickValueIteration::generateModels()
{
  for (int x = 0; x < xSteps; x++)
  {
    for (int y = 0; y < ySteps; y++)
    {
      for (auto& entry : kickTemplate)
      {
        auto action = entry.first;
        Model model;
        std::map<State*, double> count;
        double X = accuracy * x;
        double Y = accuracy * y;

        for (auto& possibilityTpl : entry.second)
        {
          double p = possibilityTpl.first;
          auto pos = possibilityTpl.second;
          double kickX = X + accuracy * (pos.first);
          double kickY = Y + accuracy * (pos.second);
          auto state = stateFor(kickX, kickY);

          if (state != NULL)
          {
            count[state] += p;
          }
          else
          {
            // Target state is NULL, we are out of field, try to check if we scored a goal

            if (kickX > Constants::field.field_length)
            {
              double dY = (kickY - Y) / (kickX - X);
              double yIntersect = Y + (Constants::field.field_length - X) * dY;
              yIntersect -= Constants::field.field_width / 2;

              // Goal
              if (fabs(yIntersect) < (Constants::field.goal_width * 0.95) / 2)
              {
                // Assigning the success state keeping in memory the kick orientation
                double kickOrientation = atan2(kickY - Y, kickX - X);
                if (kickOrientation < 0)
                {
                  kickOrientation += 2 * M_PI;
                }
                kickOrientation = kickOrientation * 180.0 / M_PI;
                State* successState = &successStates[floor(kickOrientation / angleAccuracy)];

                count[successState] += p;
              }
              else
              {
                count[&failState] += p;
              }
            }
            else
            {
              count[&failState] += p;
            }
          }
        }

        for (auto& entry : count)
        {
          model.push_back(Possibility(entry.second, entry.first));
        }

        states[x][y].models[action] = model;
      }
    }
  }
}

bool KickValueIteration::iterate()
{
  bool changed = false;

  for (int x = 0; x < xSteps; x++)
  {
    for (int y = 0; y < ySteps; y++)
    {
      double score = -5000;
      for (auto& entry : states[x][y].models)
      {
        double actionScore = 0;
        for (auto& possibility : entry.second)
        {
          actionScore +=
              possibility.first *
              (rewardFor(&states[x][y], possibility.second, kickLengths[entry.first.kick]) + possibility.second->score);
        }

        if (actionScore > score)
        {
          score = actionScore;
        }
      }

      if (score > states[x][y].score)
      {
        changed = true;
        states[x][y].score = score;
      }
    }
  }

  return changed;
}

KickValueIteration::State* KickValueIteration::stateForFieldPos(double x, double y)
{
  // Capping the position in the field
  x += Constants::field.field_length / 2.0;
  y += Constants::field.field_width / 2.0;

  if (x < 0)
    x = 0;
  if (x > Constants::field.field_length)
    x = Constants::field.field_length;
  if (y < 0)
    y = 0;
  if (y > Constants::field.field_width)
    y = Constants::field.field_width;

  return stateFor(x, y);
}

KickValueIteration::State* KickValueIteration::stateFor(double x, double y)
{
  int X = round(x / accuracy);
  int Y = round(y / accuracy);

  if (states.count(X))
  {
    if (states[X].count(Y))
    {
      return &states[X][Y];
    }
  }

  return NULL;
}

std::vector<std::string> KickValueIteration::getKickNames()
{
  return { "classic", "small" };

  // return { "lateral", "classic", "small" };
}

void KickValueIteration::loadScores(KickStrategy& strategy)
{
  // Generate the states
  generateStates();

  // Genrate kick templates
  generateTemplate();

  // Generate the model of actions for each state
  generateModels();

  for (int y = 0; y < ySteps; y++)
  {
    for (int x = 0; x < xSteps; x++)
    {
      states[x][y].score = strategy.scoreFor(states[x][y].fieldX - Constants::field.field_length / 2.0,
                                             states[x][y].fieldY - Constants::field.field_width / 2.0);
    }
  }
}