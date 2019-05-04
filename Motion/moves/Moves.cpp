#include "moves/Moves.h"

#include "Walk.h"
#include "Kick.h"
#include "StandUp.h"
#include "IMUTest.h"
#include "Replayer.hpp"
#include "Penalty.h"
#include "Head.h"
#include "Robocup.h"
#include "Playing.h"
#include "Search.h"
#include "GoalKeeper.h"
#include "Placer.h"
#include "ApproachPotential.h"
#include "LogMachine.hpp"
#include "GoalKick.hpp"
#include "AutonomousPlaying.h"
#include "OldWalk.h"

#include "ReactiveKicker.h"

#include "QKickController.h"
#include "ClearingKickController.h"
#include "PenaltyKickController.h"
#include "policies/expert_approach.h"
#include "problems/extended_problem_factory.h"
#include "rhoban_csa_mdp/core/policy_factory.h"

// #include "KickPhilipp.hpp"
// #include "KickCalibration.hpp"
// #include "LateralStep.hpp"
// #include "OdometryCalibration.hpp"
// #include "ModelCalibration.hpp"
// #include "LearnedApproach.h"
// #include "MDPKickController.h"

Moves::Moves(MoveScheduler* scheduler) : _scheduler(scheduler)
{
  if (_scheduler == nullptr)
  {
    throw std::logic_error("Moves null pointer");
  }

  // Loading all Moves
  Kick* kick = new Kick;
  add(kick);
  // Forcing generation of kick motions at kick creation
  kick->cmdKickGen();

  Walk* walk = new Walk(kick);
  OldWalk* oldWalk = new OldWalk(kick);
  Head* head = new Head;
  Placer* placer = new Placer(walk);
  StandUp* standup = new StandUp;
  add(oldWalk);
  add(standup);
  add(head);
  add(new Search(walk, placer));
  add(new ApproachPotential(walk));
  add(placer);
  // add(lateralStep);

  add(new GoalKeeper(walk, placer));
  add(new Robocup(walk, standup, placer));
  add(new PlayingMove(walk));

  // Dev moves
  add(new IMUTest);
  add(new Replayer);

  add(new LogMachine(walk, head));

  // add(new KickCalibration(approach));
  add(new GoalKick());

  // Requires additionnal dependencies

  add(new QKickController());
  add(new ClearingKickController());
  auto penaltyController = new PenaltyKickController();
  add(penaltyController);
  add(new Penalty(penaltyController));

  add(new ReactiveKicker(walk));
  add(new AutonomousPlaying(walk, standup));
  add(walk);

  //    csa_mdp::PolicyFactory::registerExtraBuilder("ExpertApproach", []() {return std::unique_ptr<csa_mdp::Policy>(new
  //    csa_mdp::ExpertApproach);}); add(new MDPKickController()); LateralStep *lateralStep = new LateralStep(); add(new
  //    LearnedApproach(walk)); add(new KickPhilipp); add(new OdometryCalibration(walk)); add(new ModelCalibration);
}

Moves::~Moves()
{
  for (const auto& it : _container)
  {
    delete it.second;
  }
}

bool Moves::hasMove(const std::string& name) const
{
  return (_map.count(name) > 0);
}
const Move* Moves::getMove(const std::string& name) const
{
  if (!hasMove(name))
  {
    throw std::logic_error("Moves move does not exist: " + name);
  }
  else
  {
    return _map.at(name);
  }
}
Move* Moves::getMove(const std::string& name)
{
  if (!hasMove(name))
  {
    throw std::logic_error("Moves move does not exist: " + name);
  }
  else
  {
    return _map.at(name);
  }
}
const std::vector<std::pair<std::string, Move*>>& Moves::getAllMoves()
{
  return _container;
}

void Moves::add(Move* move)
{
  _container.push_back({ move->getName(), move });
  _map.insert({ move->getName(), move });
  // Assign the MoveScheduler instance
  move->setScheduler(_scheduler);
}
