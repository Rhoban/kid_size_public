#pragma once

#include <map>
#include <vector>
#include <json/json.h>
#include "KickStrategy.hpp"
#include "CorridorProfile.hpp"
#include <kick_model/kick_model_collection.h>

class KickValueIteration
{
public:
  // TODO: This class should be parsed instead of having so many parameters
  KickValueIteration(std::string kicksFile, double accuracy = 0.2, double angleAccuracy = 5,
                     bool dump = false, double tolerance = 5, double grassOffset = 180);

  // Actions are discrete ints
  struct Action
  {
    std::string kick;
    double orientation;

    inline bool operator<(const Action& other) const
    {
      if (kick == other.kick)
      {
        return orientation < other.orientation;
      }
      else
      {
        return kick < other.kick;
      }
    }
  };

  // In a given state, you ahve a probability to reach another state
  struct State;
  typedef std::pair<double, State*> Possibility;

  // A model is the probability distribution of each state that
  // can be reached
  typedef std::vector<Possibility> Model;

  // A state and the model of the actions from this state
  struct State
  {
    double score;
    int x, y;
    double fieldX, fieldY;
    std::map<Action, Model> models;
  };

  // For a given orientation, you have a probability to reach a certain box
  typedef std::pair<double, std::pair<int, int>> PossibilityTemplate;

  // For each action, the probabilities (sum=1) to reach a set of targets
  std::map<Action, std::vector<PossibilityTemplate>> kickTemplate;

  KickStrategy generate();

  KickStrategy::Action bestAction(State *state);

  Json::Value toJson();

protected:
  csa_mdp::KickModelCollection kicks;
  double accuracy;
  double angleAccuracy;
  bool dump;
  double tolerance;

  // Fail & success states
  State failState, successState;

  // Discrete steps for x, y and alpha
  int xSteps, ySteps, aSteps;

  // State for a x, y box
  std::map<int, std::map<int, State>> states;

  // Generating steps
  void generateStates();
  void generateTemplate();
  void generateModels();
  bool iterate();

  // Reward function from a state to another one
  double rewardFor(State* from, State* state);

  // State for a given x/y
  State* stateFor(double x, double y);

  // Allowed kick names
  std::vector<std::string> getKickNames();
};
