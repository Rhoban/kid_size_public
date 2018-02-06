#pragma once

#include "moves/KickController.h"
#include <rhoban_utils/control/control.h>
#include <services/TeamPlayService.h>

#include "rhoban_csa_mdp/core/policy.h"
#include "problems/kick_controler.h"//from csa_mdp_experiments

#include "rhoban_utils/angle.h"

class Walk;

class MDPKickController : public KickController
{
public:
  MDPKickController();
  std::string getName();

  void onStart();
  void onStop();

  void step(float elapsed);

protected:
  // Return current state, requesting other moves and services
  Eigen::VectorXd getState();

  // Update current policy from json
  void updatePolicy();

  // Update problem properties from json
  void updateProblem();

  void displayProblemLimits();

  /// The target is updated every 'update_period' seconds
  /// This parameter ensures that the kick direction will not oscillate
  double update_period;

  /// Elapsed time since last target update in seconds
  double time_since_update;

  // Optimal policy
  std::unique_ptr<csa_mdp::Policy> policy;

  // Properties of the problem
  csa_mdp::KickControler problem;

  // File name for policy
  std::string policy_file;

  // File name for problem
  std::string problem_file;
};
