#include "MDPKickController.h"

#include "Walk.h"
#include "scheduler/Helpers.h"

#include "rhoban_utils/logging/logger.h"
#include <services/LocalisationService.h>
#include <services/ModelService.h>
#include <fstream>
#include <stdexcept>

#include "rhoban_csa_mdp/core/policy_factory.h"

#include "policies/expert_approach.h"
#include "policies/ok_seed.h"
#include "rhoban_csa_mdp/core/policy_factory.h"


#include "robocup_referee/constants.h"

using csa_mdp::ExpertApproach;
using csa_mdp::Policy;
using csa_mdp::PolicyFactory;

using namespace rhoban_utils;
using namespace rhoban_geometry;
using robocup_referee::Constants;

static rhoban_utils::Logger logger("MDPKickController");

MDPKickController::MDPKickController()
{
  Move::initializeBinding();
  initBindings();

  // Binding commands
  bind->bindFunc("kick_controler_update_policy", "Update the policy used for kick_controler",
                 &MDPKickController::updatePolicy, *this);
  bind->bindFunc("kick_controler_update_problem",
                 "Update the problem used for kick_controler",
                 &MDPKickController::updateProblem, *this);
  // Bind members
  bind->bindNew("policy_file", policy_file, RhIO::Bind::PullOnly)
    ->defaultValue("KickControllerPolicy.json")
    ->persisted(true);
  bind->bindNew("problem_file", problem_file, RhIO::Bind::PullOnly)
    ->defaultValue("KickControllerProblem.json")
    ->persisted(true);
  bind->bindNew("update_period", update_period, RhIO::Bind::PullOnly)
    ->defaultValue(5.0)
    ->persisted(true);
  // Bind monitorable values
  bind->bindNew("time_since_update", time_since_update, RhIO::Bind::PushOnly)
    ->defaultValue(0);

  bind->pull();

  // Add custom policies
  PolicyFactory::registerExtraBuilder("OKSeed",
                                      []() {return std::unique_ptr<Policy>(new OKSeed);});


  updatePolicy();
  updateProblem();
}

std::string MDPKickController::getName()
{
  return "kick_controler";
}

void MDPKickController::onStart()
{
  bind->pull();
  
  logger.log("MDPKickController Starting: policy '%s' and problem '%s'",
             policy_file.c_str(),
             problem_file.c_str());

  // Ensuring that policy has been loaded and resetting it (it does matter for expert approach)
  if (!policy) throw std::logic_error("MDPKickController: Policy is not loaded yet");
  policy->init();
}

void MDPKickController::onStop()
{
}

void MDPKickController::step(float elapsed)
{
  bind->pull();

  time_since_update += elapsed;

  // Waiting 'update_period' before updating actions
  if (time_since_update < update_period) {
    bind->push();
    return;
  }

  time_since_update = 0;

  Eigen::VectorXd state = getState();
  Eigen::VectorXd action = policy->getAction(state);

  int action_id = action(0);

  int kicker_id(0), kick_option(0);
  problem.analyzeActionId(action_id, &kicker_id, &kick_option);

  if (kicker_id > 0) {
    throw std::runtime_error("kicker_id > 0 not handled actually");
  }

  /// Simple correspondance for now
  allowed_kicks = problem.getAllowedKicks(kicker_id, kick_option);
  kick_dir = rad2deg(problem.getKickDir(state, action));

  /// If ball is outside of bound use simple custom actions
  double ball_x = state(0);
  double ball_y = state(1);
  if (ball_x > Constants::field.fieldLength / 2) {
    // Ball is outside of opponent line
    double y_limit = (Constants::field.goalWidth / 2);
    if (std::fabs(ball_y) < y_limit) {
      // Ball is roughly centered -> shoot toward goal
      kick_dir = 0;
      allowed_kicks = {"classic", "lateral"};
    }
    else {
      // Ball is not centered -> center the ball slightly behind
      kick_dir = ball_y > 0 ? -120 : 120;
      allowed_kicks = {"classic", "lateral"};
    }
  }

  bind->push();
}

Eigen::VectorXd MDPKickController::getState()
{
  // Access localisation service
  auto loc = getServices()->localisation;
  // Getting ball properties
  Point ball_pos   = loc->getBallPosField();
  Point robot_pos  = loc->getFieldPos();
  double robot_dir = loc->getFieldOrientation();
  // Building current state
  Eigen::VectorXd state(5);
  state[0] = ball_pos.getX();
  state[1] = ball_pos.getY();
  state[2] = robot_pos.getX();
  state[3] = robot_pos.getY();
  state[4] = robot_dir;

  return state;
}

void MDPKickController::updatePolicy()
{
  bind->pull();
  logger.log("Loading policy at '%s'", policy_file.c_str());
  try {
    policy = PolicyFactory().buildFromJsonFile(policy_file);
    policy->setActionLimits(problem.getActionsLimits());
    //displayProblemLimits();
  }
  catch (const std::runtime_error & exc) {
    logger.warning("%s",exc.what());
  }
}

void MDPKickController::updateProblem()
{
  bind->pull();
  logger.log("Loading problem at '%s'", problem_file.c_str());
  try {
    problem = csa_mdp::KickControler();
    problem.loadFile(problem_file);
    if (problem.getNbPlayers() > 1) {
      throw std::runtime_error("MDPKickController::updateProblem: problem has more than 1 player");
    }
    if (policy)
    {
      policy->setActionLimits(problem.getActionsLimits());
    }
    //displayProblemLimits();
  }
  catch (const std::runtime_error & exc) {
    logger.warning("%s", exc.what());
  }
}

void MDPKickController::displayProblemLimits()
{
  std::cout << "StateLimits" << std::endl;
  std::cout << problem.getStateLimits() << std::endl;
  std::cout << "ActionLimits" << std::endl;
  for (int action_id = 0; action_id < problem.getNbActions(); action_id++) {
    std::cout << "-> " << action_id << ": " << std::endl
              << problem.getActionLimits(action_id) << std::endl;
  }
}
