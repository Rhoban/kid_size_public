#include <rhoban_utils/timing/time_stamp.h>
#include <services/TeamPlayService.h>
#include <strategy/PlacementOptimizer.h>
#include <services/RefereeService.h>
#include <unistd.h>
#include "CaptainService.h"

using namespace rhoban_geometry;

std::string CaptainService::Config::getClassName() const
{
    return "CaptainConfig";
}

void CaptainService::Config::fromJson(const Json::Value & json_value, const std::string & dir_name)
{
    basePositions.clear();
    if (json_value.isMember("positions")) {
        for (auto position : json_value["positions"]) {
            BasePosition basePosition;
            basePosition.targetPosition = Point(position["x"].asFloat(), position["y"].asFloat());
            basePosition.targetOrientation = position["orientation"].asFloat();
            basePosition.priority = position["priority"].asFloat();
            
            if (position.isMember("kickerOffset")) {
                basePosition.kickerOffset = position["kickerOffset"].asFloat();
            } else {
                basePosition.kickerOffset = 0;
            }
            
            basePositions.push_back(basePosition);
        }
    }
}

Json::Value CaptainService::Config::toJson() const
{   
    throw new std::runtime_error("Can't save captain.json");
}

CaptainService::CaptainService()
: bind("captain"), captainThread(NULL)
{
    // Loading configuration file
    config.loadFile("captain.json");
    
    // Scheduling frequency
    bind.bindNew("frequency", frequency, RhIO::Bind::PullOnly)
        ->defaultValue(10)->comment("Captain refresh max frequency");
    
    // Creating UDP broadcaster
    _broadcaster = new rhoban_utils::UDPBroadcast(28646, 28646);
}

CaptainService::~CaptainService()
{
    if (captainThread != NULL) {
        running = false;
        
        captainThread->join();
        delete captainThread;
    }
}

int CaptainService::captainId()
{
    auto referee = getServices()->referee;
    auto teamPlay = getServices()->teamPlay;
    auto &info = teamPlay->allInfo();
    
    for (auto &entry : info) {
        auto &robotInfo = entry.second;
        if (entry.first < teamPlay->myId() &&  // The robot ID is smaller than mine
           !robotInfo.isOutdated() &&          // The robot info are not outdated
           robotInfo.state != rhoban_team_play::Unknown &&       // The robot is in a known state
           !referee->isPenalized(robotInfo.id)) {
               return robotInfo.id;
           }
    }
    
    return teamPlay->myId();
}

bool CaptainService::IAmCaptain()
{
    auto teamPlay = getServices()->teamPlay;
    
    return captainId() == teamPlay->myId();
}

rhoban_team_play::CaptainInfo CaptainService::getInfo()
{
    mutex.lock();
    rhoban_team_play::CaptainInfo tmp = info;
    mutex.unlock();
    
    return tmp;
}

CaptainService::Instruction CaptainService::getInstruction()
{
    auto teamPlay = getServices()->teamPlay;
    int id = teamPlay->myId();
    Instruction instruction;
    auto tmp = getInfo();
    
    if (id >= 1 && id < CAPTAIN_MAX_ID) {
        instruction.targetPosition = Point(tmp.robotTarget[id-1][0], tmp.robotTarget[id-1][1]);
        instruction.targetOrientation = tmp.robotTarget[id-1][2];
    } else {
        std::cerr << "Captain: bad ID!" << std::endl;
    }
    
    return instruction;
}

void CaptainService::setSolution(PlacementOptimizer::Solution solution)
{
    std::cout << "~" << std::endl;
    for (auto &robotTarget : solution.robotTarget) {
        auto &robot = robotTarget.first;
        auto &target = robotTarget.second;
        info.robotTarget[robot-1][0] = target.position.x;
        info.robotTarget[robot-1][1] = target.position.y;
        info.robotTarget[robot-1][2] = target.orientation;
        std::cout << "CAPTAIN: Robot #" << robot << " should go to " << target.position.x << ", "
        
        << target.position.y << std::endl;
    }    
}

void CaptainService::compute()
{
    auto referee = getServices()->referee;
    auto teamPlay = getServices()->teamPlay;
    
    // First collecting robots that are able to play
    std::map<int, rhoban_team_play::TeamPlayInfo> robots;
    std::vector<int> robotIds;
    for (auto &entry : teamPlay->allInfo()) {
        auto info = entry.second;
        if (!info.isOutdated() && // Info should not be updated
        !referee->isPenalized(info.id) &&  // Robot should not be penalized
        info.state != rhoban_team_play::Unknown // It should be playing
    ) {
            robots[info.id] = info;
            robotIds.push_back(info.id);
        }
    }
    
    if (referee->isPlacingPhase() || true) { // XXX: REMOVE TRUE
        // List the possible targets
        std::vector<PlacementOptimizer::Target> targets;
        for (auto &basePosition : config.basePositions) {
            PlacementOptimizer::Target target;
            
            target.position = basePosition.targetPosition;
            
            // If we have the kick off, kickerOffset is added to the x of the target
            // position
            if (referee->myTeamKickOff() && !referee->isDroppedBall()) {
                target.position.x += basePosition.kickerOffset;
            }
            
            target.orientation = basePosition.targetOrientation;
            target.data = basePosition.priority;
            targets.push_back(target);
        }
        
        // Finding the best solution
        auto solution = PlacementOptimizer::optimize(robotIds, targets, 
            [&robots](PlacementOptimizer::Solution solution) -> float {
                // The score is the walk duration (which is the maximum distance to walk
                // by a robot)
                // Plus the sum of the priority *1000 to prioritize the lower priority
                // values
                float score = 0;
                float penaltyScore = 0;
                for (auto &robotTarget : solution.robotTarget) {
                    auto robot = robots[robotTarget.first];
                    float walkLength = (Point(robot.fieldX, robot.fieldY) - robotTarget.second.position).getLength();
                    penaltyScore += robotTarget.second.data*1000;
                    if (score < walkLength) {
                        score = walkLength;
                    }
                }
                return penaltyScore + score;
        });
        
        setSolution(solution);
    } else {
        // We are playing    
    }
    
    info.id = teamPlay->myId();
}

bool CaptainService::tick(double elapsed)
{
    if (captainThread == NULL) {
        // Running the captain thread
        running = true;
        captainThread = new std::thread([this] {
            this->execThread();
        });
    }
    
    rhoban_team_play::CaptainInfo receive;
    size_t len = sizeof(receive);
    
    while (_broadcaster->checkMessage((unsigned char*)&receive, len)) {
        if (len != sizeof(receive)) {
            std::cout << "ERROR: CaptainService: invalid message" << std::endl;
            continue;
        }
        
        mutex.lock();
        info = receive;
        info.timestamp = rhoban_utils::TimeStamp::now().getTimeMS();
        mutex.unlock();
    }
    
    return true;
}

void CaptainService::execThread()
{
    bind.pull();
    auto lastTick = rhoban_utils::TimeStamp::now();
    auto teamPlay = getServices()->teamPlay;
    
    while (running) {
        lastTick = rhoban_utils::TimeStamp::now();
        // Updating
        if (teamPlay->isEnabled()) {
            if (IAmCaptain()) {
                mutex.lock();
                compute();   
                mutex.unlock();
                     
                // Sending captain packet
                _broadcaster->broadcastMessage((unsigned char*)&info, sizeof(info));
            }
        } else {
            compute();
        }
        
        // Sleepint if needed to fit the given frequency
        auto now = rhoban_utils::TimeStamp::now();
        double elapsed = diffSec(lastTick, now);
        double toSleep = (1.0/frequency) - elapsed;
        if (toSleep > 0) {
            usleep(round(toSleep*1000000));
        }
    }    
}