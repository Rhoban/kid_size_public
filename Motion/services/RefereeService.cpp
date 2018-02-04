#include <rhoban_utils/logging/logger.h>
#include "DecisionService.h"
#include "RefereeService.h"
#include <RhIO.hpp>

static rhoban_utils::Logger out("referee");
using namespace robocup_referee;

RefereeService::RefereeService()
{
    _state = "";
    force = false;

    bind = new RhIO::Bind("referee");
    
    bind->bindNew("state", _state, RhIO::Bind::PushOnly)
        ->comment("State of the Referee services");
    
    bind->bindNew("id", id, RhIO::Bind::PullOnly)
        ->comment("The robot ID")
        ->defaultValue(0)
        ->persisted(true)
        ;
    bind->bindNew("teamId", teamId, RhIO::Bind::PullOnly)
        ->comment("The team ID")
        ->defaultValue(0)
        ->persisted(true)
        ;
    bind->bindNew("force", force, RhIO::Bind::PullOnly)
        ->comment("Force the playing to true")
        ->defaultValue(false)
        ;
    bind->bindNew("beginDuration", beginDuration, RhIO::Bind::PullOnly)
        ->comment("Duration of the begining phase")
        ->defaultValue(15.0)
        ;
    bind->bindNew("timeSincePlaying", timeSincePlaying, RhIO::Bind::PushOnly)
        ->comment("Time elapsed since playing")
        ->defaultValue(0.0);
    bind->bindNew("timeSinceGamePlaying", timeSinceGamePlaying, RhIO::Bind::PushOnly)
        ->comment("Time elapsed since game playing")
        ->defaultValue(0.0);
    bind->bindNew("dumpGameState", dumpGameState, RhIO::Bind::PullOnly)
        ->comment("Activate dump of game status")
        ->defaultValue(false);

    bind->bindNew("alive", alive, RhIO::Bind::PullOnly)
        ->comment("Referee alive status")
        ->defaultValue(2);

    bind->bindFunc("infoPlaying", "Are we playing?",
            &RefereeService::cmdPlaying, *this);

    bind->pull();

    start();
}

RefereeService::~RefereeService()
{
    delete bind;
}

bool RefereeService::tick(double elapsed)
{
    bind->pull();
    checkPlaying();
    if (isPlaying()) {
        timeSincePlaying += elapsed;
    } else {
        timeSincePlaying = 0;
    }
    if (isGamePlaying()) {
        timeSinceGamePlaying += elapsed;
    } else {
        timeSinceGamePlaying = 0;
    }

    setState(teamId, id, alive);

    if (dumpGameState) {
        const GameState & gs = getGameState();
        std::cout << &gs << std::endl;
    }

    setTextualState();

    bind->push();

    return true;
}

int RefereeService::gameTime()
{ 
    auto gameState = getGameState();

    // No recent update from the game controller
    if (gameState.getLastUpdate() < 500) {
        return 600-remaining;
    } else {
        return -1;
    }
}

bool RefereeService::myTeamKickOff()
{
    const auto & gs = getGameState();
    int kickingTeamId = gs.getKickOffTeam();
    return kickingTeamId == teamId || kickingTeamId < 0;
}

bool RefereeService::isDroppedBall()
{
    const auto & gs = getGameState();
    int kickingTeamId = gs.getKickOffTeam();
    return kickingTeamId < 0;
}

bool RefereeService::isFreeKick()
{
    const auto & gs = getGameState();
    return gs.getSecGameState() == Constants::STATE2_DIRECT_FREE_KICK
        || gs.getSecGameState() == Constants::STATE2_INDIRECT_FREE_KICK
        || gs.getSecGameState() == Constants::STATE2_PENALTY_KICK;
}

bool RefereeService::myTeamFreeKick()
{
    const auto & gs = getGameState();
    return gs.getSecondaryTeam() == teamId;
}

bool RefereeService::isPenalized()
{
  return isPenalized(id);
}

bool RefereeService::isPenalized(int id)
{
    auto gameState = getGameState();
    for (int k=0; k<gameState.getNbTeam(); k++) {
        auto team = gameState.getTeam(k);
        if (team.getTeamNumber() == teamId) {
            int idz = id-1;
            if (idz >= 0 && idz < team.getNbRobots()) {
                auto robot = team.getRobot(idz);
                if (robot.getPenalty() != 0) {
                    return true;
                }
                if (robot.getRedCardCount() > 0) {
                    return true;
                }
            }
        }
    }

    return false;
}

bool RefereeService::shouldLetPlay()
{
    auto gameState = getGameState();

    // No recent update from the game controller
    if (gameState.getLastUpdate() < 500) {
        // The game is running for less than 10s
        if (timeSinceGamePlaying < 10) {
            // XXX: It is possible that NO team have the kick off in case of dropped  .keevi .ball
            int team = gameState.getKickOffTeam();
            // We are not the kick off team
            if (team >= 0 && team != teamId) {
                return true;
            }
        }
    }

    return false;
}

bool RefereeService::isPlaying()
{
    return playing;
}

bool RefereeService::isGamePlaying()
{
    return gamePlaying;
}

bool RefereeService::isInitialPhase()
{
    if (force) return false;
    return getGameState().getActualGameState() == Constants::STATE_INITIAL;
}

bool RefereeService::isPlacingPhase()
{
    if (force) return false;
    return getGameState().getActualGameState() == Constants::STATE_READY;
}

bool RefereeService::isFreezePhase()
{
    if (force) return false;
    return getGameState().getActualGameState() == Constants::STATE_SET
        || (isFreeKick() && getGameState().getSecondaryMode() == 1);
}

void RefereeService::checkPlaying()
{
    bind->pull();

    // This is the force flag
    if (force) {
        gamePlaying = playing = true;
        return;
    }

    // Game state
    auto gameState = getGameState();
    remaining = gameState.getEstimatedSecs();

    // If there were no update from the game controller the last
    // five seconds, suppose we are playing
    if (gameState.getLastUpdate() > 500) {
        gamePlaying = playing = true;
        return;
    }

    // Checking the game state
    if (gameState.getActualGameState() != Constants::STATE_PLAYING) {
        wasPenalized = false;
        gamePlaying = playing = false;
        return;
    }
                    
    gamePlaying = true;

    // Checking if our team is here and if the current player is not penalized
    for (int k=0; k<gameState.getNbTeam(); k++) {
        auto team = gameState.getTeam(k);
        if (team.getTeamNumber() == teamId) {
            int idz = id-1;
            if (idz >= 0 && idz < team.getNbRobots()) {
                auto robot = team.getRobot(idz);
                if (robot.getPenalty() != 0) {
                    wasPenalized = true;
                    playing = false;
                    return;
                }
            }
        }
    }

    playing = true;
}
        
double RefereeService::getTimeSincePlaying()
{
    if (!isPlaying()) {
        return 0;
    } else {
        return timeSincePlaying;
    }
}

bool RefereeService::isBegining()
{
    return isPlaying() && (timeSincePlaying < beginDuration);
}

std::string RefereeService::cmdPlaying()
{
    if (isPlaying()) {
        std::stringstream ss;
        ss << "We are playing since " << timeSincePlaying << "s." << std::endl;
        ss << "The game is playing since " << timeSinceGamePlaying << "s." << std::endl;
        ss << "Referee time: " << remaining << "." << std::endl;
        ss << "Referee last update: " << (getGameState().getLastUpdate()/100.0) << "." << std::endl;

        if (shouldLetPlay()) {
            ss << "We should not touch the ball.";
        }

        return ss.str();
    }
    else if (isInitialPhase()) {
        return "We are in initial phase";
    }
    else if (isPlacingPhase()) {
        return "We are in placing phase";
    }
    else if (isFreezePhase()) {
        return "We are in the freeze phase";
    }
    else if (isPenalized(id)) {
        return "I am penalized.";
    }
    else {
        return "We are not playing.";
    }
}

void RefereeService::resetTimer()
{
    timeSincePlaying = 0;
}
        
int RefereeService::getSecondaryTime()
{
    const auto & gs = getGameState();
    return gs.getSecondarySecs();
}
        
void RefereeService::setTextualState()
{
    if (isPlaying()) {
        if (shouldLetPlay()) {
            _state = "Let play ";
        } else {
            _state = "Playing ";
        }
    }
    else if (isInitialPhase()) {
        _state = "Initial ";
    }
    else if (isPlacingPhase()) {
        _state = "Placing ";
    }
    else if (isFreezePhase()) {
        _state = "Freeze ";
    }
    else if (isPenalized(id)) {
        _state =  "Penalized ";
    }
    else {
        _state = "Not playing ";
    }
    _state += std::to_string(getGameState().getLastUpdate()/100.0);
}

