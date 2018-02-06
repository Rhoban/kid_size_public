#include "LearnedApproach.h"

#include "KickController.h"
#include "Walk.h"
#include "scheduler/Helpers.h"

#include "rhoban_utils/logging/logger.h"
#include <services/LocalisationService.h>
#include <services/ModelService.h>
#include <fstream>
#include <stdexcept>

#include "rhoban_csa_mdp/core/policy_factory.h"
#include "problems/extended_problem_factory.h"

#include "policies/expert_approach.h"
#include "policies/mixed_approach.h"

using csa_mdp::ExpertApproach;
using csa_mdp::MixedApproach;
using csa_mdp::Policy;
using csa_mdp::PolicyFactory;

using namespace rhoban_utils;

static rhoban_utils::Logger logger("LearnedApproach");

LearnedApproach::LearnedApproach(Walk * walk_)
  : ApproachMove(walk_)
{
  csa_mdp::ExtendedProblemFactory::registerExtraProblems();

  Move::initializeBinding();
  initBindings();

  kick_solutions["classic"    ] = KickSolution();
  kick_solutions["lateral"    ] = KickSolution();
  kick_solutions["small"      ] = KickSolution();
  kick_solutions["opportunist"] = KickSolution();

  // Binding commands
  bind->bindFunc("learned_approach_update_policies",
                 "Update the policies used for learned_approach",
                 &LearnedApproach::updatePolicies, *this);
  bind->bindFunc("learned_approach_update_problems",
                 "Update the problems used for learned_approach",
                 &LearnedApproach::updateProblems, *this);
  // Bind members
  bind->bindNew("time", time, RhIO::Bind::PushOnly)->defaultValue(-1.0);
  bind->bindNew("log_file", log_file, RhIO::Bind::PullOnly)
    ->defaultValue("");
  // Kick related variables
  bind->bindNew("doKick", do_kick, RhIO::Bind::PullOnly)->defaultValue(true)
    ->comment("Start a kick when the approach is finish")
    ->persisted(true);
  bind->bindNew("warming", is_warming_up, RhIO::Bind::PushOnly)->defaultValue(false)
    ->comment("Is the robot warming up to prepare shoot");
  bind->bindNew("warm_up_time", warm_up_time, RhIO::Bind::PullOnly)->defaultValue(1.0)
    ->comment("How many seconds before kicking once walk has been disabled")
    ->persisted(true);
  bind->bindNew("is_kicking", is_kicking, RhIO::Bind::PushOnly)->defaultValue(false)
    ->comment("Is the robot currently kicking?");
  // Kick solutions related variables
  bind->bindNew("activeApproach", active_approach, RhIO::Bind::PushOnly)->defaultValue("none")
    ->comment("Which policy is currently used");
  for (auto & entry : kick_solutions) {
    bind->bindNew(entry.first + "_policy_file",
                  entry.second.policy_file,
                  RhIO::Bind::PullOnly)
      ->defaultValue("learned_approach/" + entry.first + "/policy.json")
      ->persisted(true);
    bind->bindNew(entry.first + "_problem_file",
                  entry.second.problem_file,
                  RhIO::Bind::PullOnly)
      ->defaultValue("learned_approach/" + entry.first + "/problem.json")
      ->persisted(true);
  }
 

  PolicyFactory::registerExtraBuilder("ExpertApproach",
                                      []() {return std::unique_ptr<Policy>(new ExpertApproach);});
  PolicyFactory::registerExtraBuilder("MixedApproach",
                                      []() {return std::unique_ptr<Policy>(new MixedApproach);});

  bind->pull();

  updatePolicies();
  updateProblems();
}

std::string LearnedApproach::getName()
{
  return "learned_approach";
}

void LearnedApproach::onStart()
{
  ApproachMove::onStart();

  bind->pull();

  time = 0;
  kick_score = 0;
  is_warming_up = false;
  // Ensuring that policy has been loaded and resetting it
  // reset does matter for expert approach
  for (const auto & entry : kick_solutions) {
    if (!entry.second.policy) {
      logger.error("Approach failed to start: no policy loaded for '%s'",
                   entry.first.c_str());
      onStop();
      throw std::logic_error("LearnedApproach: Policy is not loaded yet");
    }
    entry.second.policy->init();
  }
  // Erase log file
  if (log_file != "")
  {
    std::ofstream file(log_file, std::ofstream::trunc);
    if (!file.is_open()) 
    {
      throw std::runtime_error(
          "LearnedApproach unable to open log: " + log_file);
    }
    file.close();
  }
}

void LearnedApproach::onStop()
{
  walk->setRawOrder(0.0, 0.0, 0.0, false);
}

void LearnedApproach::step(float elapsed)
{
  bind->pull();

  active_approach = getActivePolicyName();

  if (!getActiveKick().policy) {
    logger.error("launching move while no policy is loaded for kick '%s'",
                 active_approach.c_str());
    stop();
  }

  time += elapsed;

  Eigen::VectorXd state = getState();

  if (is_kicking)
  {
    //LH: can't remember why time is there, a comment would have been nice
    if (time > 0.25 && !walk->isKicking()) {
      //TODO: eventually handle penalty case
      is_kicking = false;
      time = 0;
    }
  }
  else if (is_warming_up)
  {
    warmUp(state);
  }
  else
  {
    // Updating expected kick
    bool lateral_kick = state(2) > M_PI / 4;
    if (isKickAllowed("lateral") && lateral_kick) {
      expectedKick = "lateral";
    }
    else {
      if (isKickAllowed("classic")) expectedKick = "classic";
      else expectedKick = "small";
    }
    // Updating expected foot
    if (expectedKick == "lateral") {
      // Kick side depend on kick direction
      kickRight = state(2) > 0;
    }
    else {
      // Kick side depend on ball direction
      kickRight = state(1) <= 0;
    }
    // Updating kick score
    updateKickScore(elapsed);
    // Apply appropriated action for kick score
    if (kick_score >= 1.0)
    {
      startWarmUp();
    }
    else if (walk->isNewStep())
    {
      updateWalk(state);
    }
  }

  bind->push();
}

std::string LearnedApproach::getActivePolicyName()
{
  if (isKickAllowed("classic") && isKickAllowed("lateral"))
    return "opportunist";
  if (isKickAllowed("classic")) return "classic";
  if (isKickAllowed("lateral")) return "lateral";
  return "small";
}

LearnedApproach::KickSolution & LearnedApproach::getActiveKick()
{
  return kick_solutions.at(active_approach);
}

Eigen::VectorXd LearnedApproach::getState()
{
  // Access localisation service
  auto loc = getServices()->localisation;
  // Getting ball properties
  auto ball_pos = loc->getBallPosSelf();
  double ball_dist = ball_pos.getLength();//Distance in meters
  double ball_azimuth = deg2rad(ball_pos.getTheta().getSignedValue());//Angle in degrees
  // Getting kick angle target
  double target_angle = deg2rad(getKickCap().getSignedValue());
  // Importing last walk orders
  Eigen::Vector3d last_order = walk->getOrder().segment(0,3);
  // Building current state
  Eigen::VectorXd state(6);
  state[0] = ball_dist;
  state[1] = ball_azimuth;
  state[2] = target_angle;
  state.segment(3,3) = last_order;
  return state;
}

void LearnedApproach::warmUp(const Eigen::VectorXd & state)
{
  // Make sure that robot does not move anymore
  walk->setRawOrder(0.0, 0.0, 0.0, false);
  // Meaningless to stay in warm up if the goal is not to kick
  if (!do_kick) stop();
  // If enough time has been waited, then start kicking
  if (time > warm_up_time)
  {
    requestKick();
    is_kicking = true;
    is_warming_up = false;
    time = 0;
  }
}

void LearnedApproach::startWarmUp()
{
    logger.log("Approach time: %lf", time);
    walk->setRawOrder(0.0, 0.0, 0.0, false);
    time = 0;
    is_warming_up = true;
}

void LearnedApproach::updateWalk(const Eigen::VectorXd & state)
{
  // Importing last walk orders
  Eigen::Vector3d last_order = walk->getOrder().segment(0,3);
  // Getting deltas
  Eigen::VectorXd action = getActiveKick().policy->getAction(state);
  // Checking that action has a proper format:
  // action(0): type of action (expected 0)
  if (action.rows() != 4 || action(0) != 0) {
    std::ostringstream oss;
    oss << "Malformed action: " << action.transpose();
    throw std::runtime_error(oss.str());
  }
  action = action.segment(1,3);
  // Getting real orders
  Eigen::VectorXd new_orders = last_order + action;
  // Bounding orders
  Eigen::MatrixXd order_limits = getActiveKick().problem.getStateLimits().block(3,0,3,2);
  for (int dim = 0; dim < 3; dim++)
  {
    double min_val = order_limits(dim,0);
    double max_val = order_limits(dim,1);
    new_orders(dim) = std::max(min_val,std::min(max_val, new_orders(dim)));
  }
  // Setting orders
  walk->setRawOrder(new_orders[0], new_orders[1], new_orders[2], true);
  // Detect support swap
  if (log_file != "" && walk->isNewStep())
  {
    std::ofstream file(log_file, std::ofstream::app);
    if (!file.is_open()) 
    {
      throw std::runtime_error(
          "LearnedApproach unable to open log: " + log_file);
    }
    file 
        << state(0) << "," 
        << state(1) << "," 
        << state(2) << "," 
        << state(3) << "," 
        << state(4) << "," 
        << state(5) << "," 
        << new_orders(0) << "," 
        << new_orders(1) << "," 
        << new_orders(2) << std::endl;
    file.close();
  }
}

void LearnedApproach::updatePolicies()
{
  bind->pull();
  for (auto & entry : kick_solutions) {
    KickSolution & ks = entry.second;
    std::string path = ks.policy_file;
    logger.log("Loading policy '%s' at '%s'",
               entry.first.c_str(), path.c_str());
    try {
      ks.policy =
        PolicyFactory().buildFromJsonFile(path);
      ks.policy->setActionLimits(ks.problem.getActionsLimits());
      //displayProblemLimits();
    }
    catch (const std::runtime_error & exc) {
      logger.warning("%s", exc.what());
    }
  }
}

void LearnedApproach::updateProblems()
{
  bind->pull();
  for (auto & entry : kick_solutions) {
    KickSolution & ks = entry.second;
    std::string path = ks.policy_file;
    logger.log("Loading problem '%s' at '%s'",
               entry.first.c_str(), path.c_str());
    try {
      ks.problem = csa_mdp::BallApproach();
      ks.problem.loadFile(ks.problem_file);
      ks.problem.setMaxDist(20.0);
      if (ks.policy)
      {
        ks.policy->setActionLimits(ks.problem.getActionsLimits());
      }
      //displayProblemLimits();
    }
    catch (const std::runtime_error & exc) {
      logger.warning("%s", exc.what());
    }
  }
}

void LearnedApproach::displayProblemLimits()
{
  for (const auto & entry : kick_solutions) {
    const KickSolution & ks = entry.second;
    std::cout << "Problem " << entry.first << std::endl;
    std::cout << "StateLimits" << std::endl;
    std::cout << ks.problem.getStateLimits() << std::endl;
    std::cout << "ActionLimits" << std::endl;
    for (int action_id = 0; action_id < ks.problem.getNbActions(); action_id++) {
      std::cout << "-> " << action_id << ": " << std::endl
                << ks.problem.getActionLimits(action_id) << std::endl;
    }
  }
}
