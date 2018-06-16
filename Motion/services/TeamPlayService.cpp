#include <iostream>
#include <stdexcept>
#include <rhoban_utils/timing/time_stamp.h>
#include <rhoban_utils/logging/logger.h>

#include "DecisionService.h"
#include "LocalisationService.h"
#include "Services.h"
#include "RefereeService.h"
#include "TeamPlayService.h"

static rhoban_utils::Logger logger("teamplay_service");

using namespace rhoban_utils;
using namespace rhoban_geometry;
using namespace rhoban_team_play;

TeamPlayService::TeamPlayService() :
    _bind(nullptr),
    _broadcaster(nullptr),
    _selfInfo(),
    _allInfo(),
    _t(0.0),
    _isEnabled(true),
    _broadcastPeriod(1.0/TEAM_PLAY_FREQUENCY)

{
    //Initialize RhiO
    _bind = new RhIO::Bind("teamplay");
    _bind->bindNew("enable", _isEnabled, RhIO::Bind::PullOnly)
        ->defaultValue(true);
    _bind->bindNew("broadcastPeriod", _broadcastPeriod, RhIO::Bind::PullOnly)
        ->comment("UDP broadcast period in seconds")
        ->defaultValue(0.2);
    _bind->bindFunc("team", "Display information about teamplay",
        &TeamPlayService::cmdTeam, *this);

    _bind->bindNew("refereeRadius", refereeRadius, RhIO::Bind::PushOnly)
        ->minimum(0.0)->maximum(2.0)
        ->comment("Additionnal radius to the teamRadius when referee asks to let play");
    refereeRadius = 1.05;

    _bind->bindNew("timeSinceLastKick", _selfInfo.timeSinceLastKick, RhIO::Bind::PushOnly)
        ->comment("Time since I performed the last kick [s]")
        ->defaultValue(10.0);

    //Initialize UDP communication
    _broadcaster = new rhoban_utils::UDPBroadcast(TEAM_PLAY_PORT, TEAM_PLAY_PORT);
    _t = 0;

    //Initialize self info
    _selfInfo.id = 0;
    _selfInfo.state = Unknown;
    _selfInfo.ballX = 0.0;
    _selfInfo.ballY = 0.0;
    _selfInfo.ballQ = 0.0;
    _selfInfo.fieldX = 0.0;
    _selfInfo.fieldY = 0.0;
    _selfInfo.fieldYaw = 0.0;
    _selfInfo.fieldQ = 0.0;
    _selfInfo.fieldConsistency = 0.0;
    memset(_selfInfo.stateReferee , '\0', sizeof(_selfInfo.stateReferee ));
    memset(_selfInfo.stateRobocup , '\0', sizeof(_selfInfo.stateRobocup ));
    memset(_selfInfo.statePlaying , '\0', sizeof(_selfInfo.statePlaying ));
    memset(_selfInfo.stateSearch  , '\0', sizeof(_selfInfo.stateSearch  ));
    memset(_selfInfo.hardwareWarnings, '\0', sizeof(_selfInfo.hardwareWarnings));
    _selfInfo.nbObstacles = 0;
    _selfInfo.timestamp = 0.0;
}

TeamPlayService::~TeamPlayService()
{
    delete _bind;
    delete _broadcaster;
}

int TeamPlayService::myId()
{
    return Helpers::getServices()->referee->id;
}

const TeamPlayInfo& TeamPlayService::selfInfo() const
{
    return _selfInfo;
}
TeamPlayInfo& TeamPlayService::selfInfo()
{
    return _selfInfo;
}

const std::map<int, TeamPlayInfo>& TeamPlayService::allInfo() const
{
    return _allInfo;
}

bool TeamPlayService::tick(double elapsed)
{
    _bind->pull();

    if (_isEnabled) {
        //Sending informations at fixed frequency
        if (_t >= _broadcastPeriod) {
            _t -= _broadcastPeriod;
            messageSend();
        }
        _t += elapsed;

        //Receiving informations
        TeamPlayInfo info;
        size_t len = sizeof(info);
        while (_broadcaster->checkMessage((unsigned char*)&info, len)) {
            if (len != sizeof(info)) {
                std::cout << "ERROR: TeamPlayService: invalid message" << std::endl;
                continue;
            }
            //Assign reception timestamp
            info.timestamp = TimeStamp::now().getTimeMS();
            processInfo(info);
        }
    } else {
        _allInfo.clear();
    }

    _bind->push();

    return true;
}

bool TeamPlayService::isEnabled()
{
    return _isEnabled;    
}

void TeamPlayService::messageSend()
{
    _selfInfo.id = myId();

    if (_selfInfo.id != 0) {
        auto decision = getServices()->decision;
        auto loc = getServices()->localisation;
        //Setting message hour
        uint16_t ms_unused;
        rhoban_utils::Logger::getTime(_selfInfo.hour, _selfInfo.min, _selfInfo.sec, ms_unused);
        //Ball position in self
        Point ballPos = loc->getBallPosSelf();
        Point ballVel = loc->getBallSpeedSelf();
        _selfInfo.ballX = ballPos.x;
        _selfInfo.ballY = ballPos.y;
        _selfInfo.ballVelX = ballVel.x;
        _selfInfo.ballVelY = ballVel.y;
        _selfInfo.ballQ = loc->ballQ;
        _selfInfo.ballOk = decision->isBallQualityGood;
        //Robot in field
        Point fieldPos = loc->getFieldPos();
        _selfInfo.fieldX = fieldPos.x;
        _selfInfo.fieldY = fieldPos.y;
        _selfInfo.fieldYaw = loc->getFieldOrientation();
        _selfInfo.fieldQ = loc->fieldQ;
        _selfInfo.fieldConsistency = loc->fieldConsistency;
        _selfInfo.fieldOk = decision->isFieldQualityGood;
        //Playing state
        strncpy(
            _selfInfo.stateReferee,
            RhIO::Root.getStr("referee/state").c_str(),
            sizeof(_selfInfo.stateReferee));
        strncpy(
            _selfInfo.stateRobocup,
            RhIO::Root.getStr("moves/robocup/state").c_str(),
            sizeof(_selfInfo.stateRobocup));
        strncpy(
            _selfInfo.statePlaying,
            RhIO::Root.getStr("moves/playing/state").c_str(),
            sizeof(_selfInfo.statePlaying));
        strncpy(
            _selfInfo.stateSearch,
            RhIO::Root.getStr("moves/search/state").c_str(),
            sizeof(_selfInfo.stateSearch));
        strncpy(
            _selfInfo.hardwareWarnings,
            RhIO::Root.getStr("model/lowlevel_state").c_str(),
            sizeof(_selfInfo.hardwareWarnings));
        //Safe terminating string
        _selfInfo.stateReferee[sizeof(_selfInfo.stateReferee)-1] = '\0';
        _selfInfo.stateRobocup[sizeof(_selfInfo.stateRobocup)-1] = '\0';
        _selfInfo.statePlaying[sizeof(_selfInfo.statePlaying)-1] = '\0';
        _selfInfo.stateSearch[sizeof(_selfInfo.stateSearch)-1] = '\0';
        _selfInfo.hardwareWarnings[sizeof(_selfInfo.hardwareWarnings)-1] = '\0';
        // Adding obstacles to message
        std::vector<rhoban_geometry::Point> opponents = loc->getOpponentsField();
        _selfInfo.nbObstacles = std::min(opponents.size(), (size_t)MAX_OBSTACLES);
        if (opponents.size() > MAX_OBSTACLES) {
          logger.warning("Too many obstacles to broadcast");
        }
        _selfInfo.obstaclesRadius = loc->teamMatesRadius;
        for (int oppIdx  = 0; oppIdx < _selfInfo.nbObstacles; oppIdx++) {
          _selfInfo.obstacles[oppIdx][0] = opponents[oppIdx].getX();
          _selfInfo.obstacles[oppIdx][1] = opponents[oppIdx].getY();
        }

        //Send UDP broadcast
        _broadcaster->broadcastMessage(
            (unsigned char*)&_selfInfo, sizeof(_selfInfo));
    }
}

void TeamPlayService::processInfo(TeamPlayInfo info)
{
    // Updates stored informations
    _allInfo[info.id] = info;
    // Update team mates position;
    auto loc = getServices()->localisation;
    // TODO: handle localization quality
    if (info.id != myId()) {
      std::vector<Eigen::Vector2d> opponents_seen(info.nbObstacles);
      for (int idx = 0; idx < info.nbObstacles; idx++) {
        opponents_seen[idx] = Eigen::Vector2d(info.obstacles[idx][0], info.obstacles[idx][1]);
      }
      loc->updateSharedOpponents(info.id, opponents_seen);
    }
}

std::string TeamPlayService::cmdTeam()
{
    std::stringstream ss;
    ss << "There is " << _allInfo.size() << " players." << std::endl;
    if (myId() == 0) {
        ss << "WARNING: I am not a player, because my id is 0" << std::endl;
    }
    ss << std::endl;
    for (auto entry : _allInfo) {
        auto info = entry.second;
        ss << "Player #" << info.id;
        if (info.id == myId()) {
            ss << " (me)";
        }
        ss << " - last update: " << info.getAge() << "ms";
        ss << std::endl;
        if (info.state == BallHandling) {
            ss << "- Is handling the ball" << std::endl;
        } else if (info.state == Playing) {
            ss << "- Is playing" << std::endl;
        } else if (info.state == Unknown) {
            ss << "- Unknown" << std::endl;
        } else if (info.state == Inactive) {
            ss << "- Is not active" << std::endl;
        } else if (info.state == GoalKeeping) {
            ss << "- Goal Keeping" << std::endl;
        } else {
            ss << "- Unknown state (?)" << std::endl;
        }
        ss << "- Ball distance: " << info.getBallDistance()
            << ", X=" << info.ballX << " Y=" << info.ballY
            << ", quality: " << info.ballQ << std::endl;
        ss << "- Field X: " << info.fieldX << ", Y: " << info.fieldY
            << ", quality: " << info.fieldQ << std::endl;
        ss << "- State Referee: " << info.stateReferee << std::endl;
        ss << "- State RoboCup: " << info.stateRobocup << std::endl;
        ss << "- State Playing: " << info.statePlaying << std::endl;
        ss << "- State Search: " << info.stateSearch << std::endl;
        ss << "- Hardware warnings: " << info.hardwareWarnings << std::endl;
        ss << std::endl;
    }

    return ss.str();
}
