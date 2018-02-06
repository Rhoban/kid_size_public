#pragma once

#include "moves/ApproachMove.h"
#include <rhoban_utils/control/control.h>
#include <services/TeamPlayService.h>

#include "rhoban_csa_mdp/core/policy.h"
#include "problems/ball_approach.h"//from csa_mdp_experiments

class Walk;

class LearnedApproach : public ApproachMove
{
public:
  LearnedApproach(Walk *walk);
  std::string getName();

  void onStart() override;
  void onStop();

  void step(float elapsed);

protected:

  /// Each KickSolution has several properties:
  class KickSolution {
  public:
    /// Policy used for the kick
    std::unique_ptr<csa_mdp::Policy> policy;
    /// Properties of the problem
    csa_mdp::BallApproach problem;
    /// File name for policy
    std::string policy_file;
    /// File name for problem
    std::string problem_file;
  };


  std::string getActivePolicyName();

  KickSolution & getActiveKick();

  /// Return current state, requesting other moves and services
  Eigen::VectorXd getState();

  /// End the approach phase and start warm_up_phase
  void startWarmUp();

  /// If warm_up time is reached or _doKick is disabled, stop the move
  void warmUp(const Eigen::VectorXd & state);

  /// Use policy to update current orders
  void updateWalk(const Eigen::VectorXd & state);

  /// Update all policies from json files
  void updatePolicies();

  /// Update all problem properties from json files
  void updateProblems();

  void displayProblemLimits();

  /// Time spent in approach [s]
  double time;

  /// Is the robot waiting to kick
  bool is_warming_up;

  /// Is the robot currently kicking
  bool is_kicking;

  /// Warm up time
  double warm_up_time;

  /// If true, the robot start an actual Kick at the end
  /// of a approach 
  bool do_kick;

  /// If not empty, state and action is logged
  /// into the given file name at each wlak cycle
  std::string log_file;

  /// Current approach
  std::string active_approach;

  /// Default kick properties
  std::map<std::string, KickSolution> kick_solutions;
};
