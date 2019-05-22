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

KickValueIteration::KickValueIteration(std::string kickFiles, double accuracy, double angleAccuracy, bool dump,
                                       double tolerance, double grassOffset)
  : accuracy(accuracy), angleAccuracy(angleAccuracy), dump(dump), tolerance(tolerance)
{
  kicks.loadFile(kickFiles);
  kicks.setGrassConeOffset(grassOffset);
}

double KickValueIteration::rewardFor(State* from, State* state)
{
  if (state == &successState)
  {
    return 0;
  }

  if (state == &failState)
  {
    return -300;
  }

  // Distance we have to walk
  double dist = sqrt(pow(from->fieldX - state->fieldX, 2) + pow(from->fieldY - state->fieldY, 2));

  return -(10 + dist / 0.15);
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

  for (int y = 0; y < ySteps; y++)
  {
    for (int x = 0; x < xSteps; x++)
    {
      double X = accuracy * x;
      double Y = accuracy * y;
      State* state = &states[x][y];

      strategy.setAction(X, Y, bestAction(state));
    }
  }

  return strategy;
}

KickStrategy::Action KickValueIteration::bestAction(KickValueIteration::State* state)
{
  Action action;
  double bestScore = -1000;
  double bestLength = -1;
  std::set<int> possibleOrientations;
  std::set<std::string> possibleKicks;
  std::map<std::string, double> kickLength;
  for (auto& kickName : getKickNames())
  {
    auto& kickModel = kicks.getKickModel(kickName);
    auto tmp = kickModel.applyKick(Eigen::Vector2d(0, 0), 0);
    kickLength[kickName] = tmp[0];
  }

  for (auto& entry : state->models)
  {
    double actionScore = 0;
    auto tmpAction = entry.first;

    // Estimating the score of this action
    for (auto& possibility : entry.second)
    {
      actionScore += possibility.first * (rewardFor(state, possibility.second) + possibility.second->score);
    }

    // If the action is withing our time tolerance
    if (fabs(actionScore - state->score) < tolerance)
    {
      possibleKicks.insert(tmpAction.kick);
      possibleOrientations.insert(round(tmpAction.orientation * 1000));
      auto length = kickLength[tmpAction.kick];

      if (bestScore < actionScore || bestLength < 0 || bestLength < length)
      {
        bestLength = length;
        action = tmpAction;
        bestScore = actionScore;
      }

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
  resultAction.tolerance = std::max<double>(0, (possibleOrientations.size() - 1) * angleAccuracy * M_PI / 180.0);
  resultAction.orientation = action.orientation;
  resultAction.score = bestScore;

  if (possibleKicks.count("classic") && possibleKicks.count("lateral"))
  {
    resultAction.kick = "opportunist";
  }
  else
  {
    resultAction.kick = action.kick;
  }
  return resultAction;
}

void KickValueIteration::generateStates()
{
  xSteps = 1 + Constants::field.field_length / (accuracy);
  ySteps = 1 + Constants::field.field_width / (accuracy);
  aSteps = 360 / angleAccuracy;

  successState.score = 0;
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
  std::normal_distribution<double> posNoise(0, 0.3);

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

        int dX = round(sample.x / accuracy) + posNoise(generator);
        int dY = round(sample.y / accuracy) + posNoise(generator);

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
  std::default_random_engine generator;
  std::normal_distribution<double> rnd(0, 1);

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
            if (kickX > Constants::field.field_length)
            {
              double dY = (kickY - Y) / (kickX - X);
              double yIntersect = Y + (Constants::field.field_length - X) * dY;
              yIntersect -= Constants::field.field_width / 2;

              // Goal
              if (fabs(yIntersect) < (Constants::field.goal_width * 0.95) / 2)
              {
                count[&successState] += p;
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
          actionScore += possibility.first * (rewardFor(&states[x][y], possibility.second) + possibility.second->score);
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
  return { "lateral", "classic", "small" };
}