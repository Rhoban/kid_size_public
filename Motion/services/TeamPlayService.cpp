#include <iostream>
#include <stdexcept>
#include <rhoban_utils/timing/time_stamp.h>
#include <rhoban_utils/logging/logger.h>

#include "DecisionService.h"
#include "LocalisationService.h"
#include "Services.h"
#include "RefereeService.h"
#include "TeamPlayService.h"

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
    _broadcastPeriod(0.3)

{
    //Initialize RhiO
    _bind = new RhIO::Bind("teamplay");
    _bind->bindNew("enable", _isEnabled, RhIO::Bind::PullOnly)
        ->defaultValue(true)->persisted(true);
    _bind->bindNew("broadcastPeriod", _broadcastPeriod, RhIO::Bind::PullOnly)
        ->comment("UDP broadcast period in seconds")
        ->defaultValue(0.3)->persisted(true);
// TODO: solve issue with RhIO and enums
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wstrict-aliasing"
    _bind->bindNew("priority", (int&)_selfInfo.priority, RhIO::Bind::PullOnly)
        ->comment("0: low; 1: normal; 2: high Priority")
        ->defaultValue(1)->persisted(true);
#pragma GCC diagnostic pop
    _bind->bindFunc("team", "Display information about teamplay",
        &TeamPlayService::cmdTeam, *this);
    _bind->bindNew("teamRadius", teamRadius)
        ->defaultValue(100)->minimum(0.0)->maximum(1000.0)
        ->persisted(true);
    _bind->bindNew("refereeRadius", refereeRadius)
        ->defaultValue(110)->minimum(0.0)->maximum(200.0)
        ->comment("Additionnal radius to the teamRadius when referee asks to let play")
        ->persisted(true);
    _bind->bindNew("aggressivity", aggressivity, RhIO::Bind::PullOnly)
        ->defaultValue(0.75)->minimum(0.0)->maximum(1.0)
        ->comment("Is the placing aggressive ore defensive?")
        ->persisted(true);

    //Initialize UDP communication
    _broadcaster = new rhoban_utils::UDPBroadcast(27645, 27645);
    _t = 0;

    //Initialize self info
    _selfInfo.id = 0;
    _selfInfo.state = Inactive;
    _selfInfo.priority = NormalPriority;
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
        
TeamPlayState TeamPlayService::myRole()
{
    std::map<int, TeamPlayInfo> robots;

    // Collecting available robots and their states
    auto referee =  Helpers::getServices()->referee;
    for (auto &entry : _allInfo) {
        auto info = entry.second;
        if ((info.state == PlacingA || info.state == PlacingB || 
             info.state == PlacingC || info.state == PlacingD)
                && !info.isOutdated()
                && !referee->isPenalized(info.id)) {
            robots[info.id] = info;
        }
    }

    // Collecting configs
    typedef std::map<int, TeamPlayState> Config;

    std::function<std::vector<Config>(std::map<int, TeamPlayInfo>)>
    collectConfigs = [&collectConfigs](std::map<int, TeamPlayInfo> robots) -> std::vector<Config> {
        std::vector<Config> result;

        for (auto &entry : robots) {
            int id = entry.first;
            for (auto role : {PlacingA, PlacingB, PlacingC, PlacingD}) {
                if (entry.second.scoreFor(role) >= 0) {
                    auto tmp = robots;
                    tmp.erase(entry.first);
                    if (tmp.size()) {
                        auto configs = collectConfigs(tmp);
                        for (auto &config : configs) {
                            config[id] = role;
                            result.push_back(config);
                        }
                    } else {
                        Config config;
                        config[id] = role;
                        result.push_back(config);
                    }
                }
            }
        }

        return result;
    };

    auto configs = collectConfigs(robots);

    if (!robots.count(myId())) {
        return PlacingA;
    }

    auto bestConfig = configs[0];
    double best = -1;
    for (auto &config : configs) {
        double score = 0;
        double ratio = 0;
        std::map<TeamPlayState, int> count;
        count[PlacingA] = 0;
        count[PlacingB] = 0;
        count[PlacingC] = 0;
        count[PlacingD] = 0;
        for (auto entry : config) {
            int id = entry.first;
            TeamPlayState role = entry.second;
            count[role] += 1;
            score += robots[id].scoreFor(role);
            if (role != robots[id].state) score += 1;

            if (role == PlacingA || role == PlacingB) {
                ratio += 1.0/robots.size();
            }
        }

        // Not having 2 robots for the same target
        for (auto entry : count) {
            if (entry.second > 1) {
                score += 5000;
            }
        }

        score += fabs(ratio-aggressivity)*1000;

        if (score < best || best < 0) {
            best = score;
            bestConfig = config;
        }
    }

    return bestConfig[myId()];
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

    return true;
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

        //Send UDP broadcast
        _broadcaster->broadcastMessage(
            (unsigned char*)&_selfInfo, sizeof(_selfInfo));
    }
}

void TeamPlayService::processInfo(TeamPlayInfo info)
{
    _allInfo[info.id] = info;
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
        } else if (info.state == Inactive) {
            ss << "- Is not active" << std::endl;
        } else if (info.state == PlacingA) {
            ss << "- Is placing A" << std::endl;
        } else if (info.state == PlacingB) {
            ss << "- Is placing B" << std::endl;
        } else if (info.state == PlacingC) {
            ss << "- Is placing C" << std::endl;
        } else if (info.state == PlacingD) {
            ss << "- Is placing D" << std::endl;
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

TeamPlayPriority TeamPlayService::myPriority()
{
    return selfInfo().priority;
}

