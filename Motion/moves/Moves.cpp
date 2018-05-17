#include "moves/Approach.h"
#include "moves/Moves.h"

#include "Walk.h"
#include "Kick.h"
// #include "LateralStep.hpp"
#include "StandUp.h"
#include "IMUTest.h"
#include "StaticLearner.hpp"
#include "Replayer.hpp"
#include "Penalty.h"
#include "Head.h"
#include "Robocup.h"
#include "Playing.h"
#include "Search.h"
#include "TestHeadSinus.hpp"
#include "TrajectoriesPlayer.hpp"
#include "KickPhilipp.hpp"
#include "CameraCalibration.hpp"
#ifdef VISION_COMPONENT
#include "ArucoCalibration.h"
#endif
#include "OdometryCalibration.hpp"
#include "ModelCalibration.hpp"
#include "GoalKeeper.h"
#include "Placer.h"
#include "ApproachPotential.h"
#include "LogMachine.hpp"
// #include "KickCalibration.hpp"
#include "GoalKick.hpp"

#include "ReactiveKicker.h"

#include "MDPKickController.h"
#include "QKickController.h"
#include "ClearingKickController.h"
#include "PenaltyKickController.h"
#include "LearnedApproach.h"
#include "policies/expert_approach.h"
#include "problems/extended_problem_factory.h"
#include "rhoban_csa_mdp/core/policy_factory.h"

Moves::Moves(MoveScheduler* scheduler) :
        _scheduler(scheduler)
{
    if (_scheduler == nullptr) {
        throw std::logic_error("Moves null pointer");
    }

    //Loading all Moves
    Kick *kick = new Kick;
    add(kick);
    // Forcing generation of kick motions at kick creation
    kick->cmdKickGen();

    Walk *walk = new Walk(kick);
    Head *head = new Head;
    Placer *placer = new Placer(walk);
    StandUp *standup = new StandUp;
    // LateralStep *lateralStep = new LateralStep();
    add(walk);
    add(standup);
    add(head);
    add(new Search(walk, placer));
    Approach *approach = new Approach(walk, head);
    add(approach);
    add(new ApproachPotential(walk));
    add(placer);
    // add(lateralStep);

    add(new GoalKeeper(walk, placer));
    add(new Robocup(walk, standup, placer));
    add(new PlayingMove(walk));

    // Dev moves
    add(new CameraCalibration);
#ifdef VISION_COMPONENT
    add(new ArucoCalibration);
#endif
    add(new OdometryCalibration(walk));
    add(new ModelCalibration);
    add(new TrajectoriesPlayer);
    add(new IMUTest);
    add(new TestHeadSinus);
    add(new StaticLearner);
    add(new Replayer);

    add(new KickPhilipp);

    add(new LogMachine(walk, head));


    // add(new KickCalibration(approach));
    add(new GoalKick());

    // Requires additionnal dependencies
    csa_mdp::PolicyFactory::registerExtraBuilder("ExpertApproach",
                                                 []() {return std::unique_ptr<csa_mdp::Policy>(new csa_mdp::ExpertApproach);});

    add(new MDPKickController());
    add(new QKickController());
    add(new ClearingKickController());
    auto penaltyController = new PenaltyKickController();
    add(penaltyController);
    add(new LearnedApproach(walk));
    add(new Penalty(penaltyController));

    add(new ReactiveKicker(walk));
}

Moves::~Moves()
{
    for (const auto& it : _container) {
    delete it.second;
    }
    }

    bool Moves::hasMove(const std::string& name) const
    {
    return (_map.count(name) > 0);
    }
    const Move* Moves::getMove(const std::string& name) const
    {
    if (!hasMove(name)) {
    throw std::logic_error(
    "Moves move does not exist: " + name);
    } else {
    return _map.at(name);
    }
    }
    Move* Moves::getMove(const std::string& name)
    {
    if (!hasMove(name)) {
    throw std::logic_error(
    "Moves move does not exist: " + name);
    } else {
    return _map.at(name);
    }
    }
    const std::vector<std::pair<std::string, Move*>>& Moves::getAllMoves()
    {
    return _container;
    }

    void Moves::add(Move* move)
    {
    _container.push_back({move->getName(), move});
    _map.insert({move->getName(), move});
    //Assign the MoveScheduler instance
    move->setScheduler(_scheduler);
    }
