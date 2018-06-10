#include <rhoban_utils/timing/time_stamp.h>
#include <services/TeamPlayService.h>
#include <strategy/PlacementOptimizer.h>
#include <services/RefereeService.h>
#include <robocup_referee/constants.h>
#include <unistd.h>
#include "CaptainService.h"

#include <rhoban_geometry/point_cluster.h>
#include <rhoban_utils/logging/logger.h>
#include <rhoban_utils/util.h>

using namespace rhoban_utils;
using namespace robocup_referee;
using namespace rhoban_geometry;

static Logger logger("CaptainService");

/**
 * Helper to bound a position avoiding the penalty area
 */
static void boundPosition(Point &point)
{
    double xMax = Constants::field.fieldLength/2 - Constants::field.goalAreaLength - 0.5;
    double yMax = Constants::field.goalAreaWidth/2;

    if (point.x > xMax) {
        point.x = xMax;
        if (point.y > yMax) point.y = yMax;
        if (point.y < -yMax) point.y = -yMax;
    }
    if (point.x < -xMax) {
        point.x = -xMax;
        point.x = -xMax;
        if (point.y > yMax) point.y = yMax;
        if (point.y < -yMax) point.y = -yMax;
    }
}

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
            basePosition.kickOff = position["kickOff"].asBool();
            
            if (position.isMember("mandatory")) {
                basePosition.mandatory = position["mandatory"].asBool();
            } else {
                basePosition.mandatory = false;
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
    
        
    bind.bindNew("passPlacingRatio", passPlacingRatio, RhIO::Bind::PullOnly)
        ->defaultValue(0.85)->comment("Ratio to the kick vector to place");
    
    bind.bindNew("passPlacingOffset", passPlacingOffset, RhIO::Bind::PullOnly)
        ->defaultValue(0.35)->comment("Offset to kick vector to place [m]")->persisted(true);
        
    bind.bindNew("perpendicularBallDistance", perpendicularBallDistance, RhIO::Bind::PullOnly)
        ->defaultValue(0.8)->comment("Distance [m] on the perpendicular to attack placement");
        
    bind.bindNew("placingBallDistance", placingBallDistance, RhIO::Bind::PullOnly)
        ->defaultValue(2.0)->comment("Distance [m] to the ball for team play placement");
        
    bind.bindNew("avoidRadius", avoidRadius, RhIO::Bind::PullOnly)
        ->defaultValue(1.0)->comment("Radius [m] to avoid colliding the ball while placing");
        
    bind.bindNew("aggressivity", aggressivity, RhIO::Bind::PullOnly)
        ->defaultValue(0.75)->minimum(0.0)->maximum(1.0)
        ->comment("Is the placing aggressive ore defensive?");
    
    bind.bindNew("captainId", captainId, RhIO::Bind::PushOnly);
    bind.bindNew("IAmCaptain", IAmCaptain, RhIO::Bind::PushOnly);

    bind.bindNew("commonBallTol", commonBallTol, RhIO::Bind::PullOnly)
      ->defaultValue(1.0)->comment("Distance [m] for merging ball candidates");

    bind.bindNew("commonBallNbRobots", info.common_ball.nbRobots, RhIO::Bind::PushOnly);
    bind.bindNew("commonBallX", info.common_ball.x, RhIO::Bind::PushOnly);
    bind.bindNew("commonBallY", info.common_ball.y, RhIO::Bind::PushOnly);
    
    // Creating UDP broadcaster
    _broadcaster = new rhoban_utils::UDPBroadcast(CAPTAIN_PORT, CAPTAIN_PORT);
}

CaptainService::~CaptainService()
{
    if (captainThread != NULL) {
        running = false;
        
        captainThread->join();
        delete captainThread;
    }
}

int CaptainService::findCaptainId()
{
    auto referee = getServices()->referee;
    auto teamPlay = getServices()->teamPlay;
    auto &info = teamPlay->allInfo();
    share = true;
    
    for (auto &entry : info) {
        auto &robotInfo = entry.second;
        if (!robotInfo.isOutdated() &&          // The robot info are not outdated
           robotInfo.state != rhoban_team_play::Unknown &&       // The robot is in a known state
           !referee->isPenalized(robotInfo.id)) {
               return robotInfo.id;
           }
    }
    
    share = false;
    return teamPlay->myId();
}

bool CaptainService::amICaptain()
{
    auto teamPlay = getServices()->teamPlay;
    
    return findCaptainId() == teamPlay->myId();
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
        instruction.order = tmp.order[id-1];
    } else {
        std::cerr << "Captain: getInstruction bad ID!" << std::endl;
    }
    
    return instruction;
}

void CaptainService::setSolution(PlacementOptimizer::Solution solution)
{
    // std::cout << "~" << std::endl;
    for (auto &robotTarget : solution.robotTarget) {
        auto &robot = robotTarget.first;
        auto &target = robotTarget.second;
        info.robotTarget[robot-1][0] = target.position.x;
        info.robotTarget[robot-1][1] = target.position.y;
        info.robotTarget[robot-1][2] = target.orientation;
        info.order[robot-1] = rhoban_team_play::CaptainOrder::Place;
        // std::cout << "CAPTAIN: Robot #" << robot << " should go to " << target.position.x << ", " << target.position.y << std::endl;
    }    
}

void CaptainService::updateCommonBall()
{
  // Gather balls which are currently seen by other robots
  std::vector<Point> balls;// (x,y, quality)
  for (const auto & robot_entry : robots) {
    const rhoban_team_play::TeamPlayInfo & info = robot_entry.second;
    logger.log("Ball info from %d: %s", info.id, info.ballOk ? "OK" : "KO");

    if (info.ballOk) {
      balls.push_back(Point(info.ballX, info.ballY));
    }
  }

  logger.log("Nb balls: %d", balls.size());

  if (balls.size() == 0) {
    info.common_ball.x = 0;
    info.common_ball.y = 0;
    info.common_ball.nbRobots = 0;
    return;
  }

  // Place balls inside clusters
  // Disclaimer: order of balls can have an influence on cluster composition
  std::vector<PointCluster> ball_clusters = createClusters(balls, commonBallTol);

  logger.log("Nb clusters: %d", ball_clusters.size());

  // Choosing best cluster with the following criteria (by priority order)
  // 1. Size of the cluster
  // 2. Distance to last common ball if applicable (if clusters have same size)
  // 3. First in list (if other criteria were not sufficient (unlikely but can still happen)
  double max_dist = 15.0;// [m]
  double best_cluster_score = 0;
  int best_cluster_id = -1;

  for (size_t cluster_id = 0; cluster_id < ball_clusters.size(); cluster_id++) {
    const PointCluster & cluster = ball_clusters[cluster_id];
    double cluster_score = cluster.size() * max_dist;
    if (info.common_ball.nbRobots > 0) {
      Point old_pos(info.common_ball.x, info.common_ball.y);
      cluster_score -= cluster.getAverage().getDist(old_pos);
    }
    if (cluster_score > best_cluster_score) {
      best_cluster_score = cluster_score;
      best_cluster_id = cluster_id;
    }
  }

  // Explicit error in case someone changes scoring or conditions
  if (best_cluster_id == -1) {
    throw std::logic_error(DEBUG_INFO + "no best cluster found, should never happen");
  }

  const PointCluster & best_cluster = ball_clusters[best_cluster_id];
  Point common_pos = best_cluster.getAverage();
  info.common_ball.nbRobots = best_cluster.size();
  info.common_ball.x = common_pos.x;
  info.common_ball.y = common_pos.y;

  logger.log("Nb robots: %d", info.common_ball.nbRobots);
}

void CaptainService::computeBasePositions()
{
    auto referee = getServices()->referee;
    bool kickOff = referee->myTeamKickOff() && !referee->isDroppedBall();
    
    // List the possible targets (filters the base positions according
    // to the kickOff flag)
    std::vector<PlacementOptimizer::Target> targets;
    for (auto &basePosition : config.basePositions) {
        if (basePosition.kickOff == kickOff) {            
            PlacementOptimizer::Target target;
            target.position = basePosition.targetPosition;
            target.orientation = basePosition.targetOrientation;
            target.mandatory = basePosition.mandatory;
            targets.push_back(target);
        }
    }
    
    // Finding the best solution
    auto solution = PlacementOptimizer::optimize(robotIds, targets, 
        [this](PlacementOptimizer::Solution solution) -> float {
            float score = 0;
            for (auto &robotTarget : solution.robotTarget) {
                auto robot = robots[robotTarget.first];
                float walkLength = (Point(robot.fieldX, robot.fieldY) - robotTarget.second.position).getLength();
                if (robotTarget.second.mandatory) {
                    walkLength *= 100;
                }
                score += walkLength;
            }
            return score;
    });
    setSolution(solution);
}

std::vector<PlacementOptimizer::Target> CaptainService::getTargetPositions(Point ball,
    Point ballTarget)
{
    std::vector<PlacementOptimizer::Target> targets;
    std::vector<PlacementOptimizer::Target> finalTargets;
    
    Point corner;
    Point vect = (ballTarget-ball)*passPlacingRatio 
        - (ballTarget-ball).normalize()*passPlacingOffset;
    Point nVect = vect.perpendicular().normalize(perpendicularBallDistance);
    bool backward = ballTarget.x-0.1 <= ball.x;
    auto oppositeCorner = corner;
    oppositeCorner.y *= -1;

    {
        PlacementOptimizer::Target target;
        
        corner = Point(Constants::field.fieldLength/2, Constants::field.goalWidth/2 + 0.25);

        if (backward) {
            target.position = ball+(ballTarget-ball)*1.15;
        } else {
            target.position = ball+vect+nVect;
        }    
        target.data = 1;
        target.orientation = (ballTarget-target.position).getTheta().getSignedValue();
        targets.push_back(target);
    }
    {
        PlacementOptimizer::Target target;
        corner = Point(Constants::field.fieldLength/2, -Constants::field.goalWidth/2 - 0.25);

        if (backward) {
            auto tmp1 = ball+vect+nVect;
            auto tmp2 = ball+vect+nVect;
            if (tmp1.x < tmp2.x) target.position = tmp1;
            else target.position = tmp2;
        } else {
            target.position = ball+vect-nVect;
        }
        target.data = 1;
        target.orientation = (ballTarget-target.position).getTheta().getSignedValue();
        targets.push_back(target);
    }
    
    {
        PlacementOptimizer::Target target;
        corner = Point(-Constants::field.fieldLength/2, Constants::field.goalWidth/2 + 0.25);
        target.position = ball+(corner-ball).normalize(placingBallDistance);
        
        target.data = 0;
        target.orientation = Angle::weightedAverage((ball-target.position).getTheta(), 0.5, 
            (oppositeCorner-target.position).getTheta(), 0.5).getSignedValue();
        targets.push_back(target);
    }
    
    {
        PlacementOptimizer::Target target;
        corner = Point(-Constants::field.fieldLength/2, -Constants::field.goalWidth/2 - 0.25);
        target.position = ball+(corner-ball).normalize(placingBallDistance);
        
        target.data = 0;
        target.orientation = Angle::weightedAverage((ball-target.position).getTheta(), 0.5, 
            (oppositeCorner-target.position).getTheta(), 0.5).getSignedValue();
        targets.push_back(target);
    }

    for (auto &target : targets) {
        // Avoiding penalty areas
        boundPosition(target.position);

        // Avoiding being too close to the ball
        if ((target.position-ball).getLength() < avoidRadius) {
            continue;
        }

        // Target orientation
        finalTargets.push_back(target);
    }

    return finalTargets;
}

void CaptainService::computePlayingPositions()
{
    int handler = -1;
    float smallerDist = 0.0;
    Point ball, ballTarget;
    
    // XXX: We should check if the goal keepr is handling the ball, because in
    // this case we should not give the handle ball order to another robot (might
    // be 2 simultaneous attackers)
    for (auto &entry : robots) {
        auto &robot = entry.second;
        if (robot.ballOk) {
            float dist = sqrt(pow(robot.ballX, 2) + pow(robot.ballY, 2));
            
            if (handler == -1 || dist < smallerDist) {
                handler = robot.id;
                smallerDist = dist;
                
                float a = robot.fieldYaw;
                
                // XXX: Smooth those points ?
                ball = Point(
                    robot.fieldX + cos(a)*robot.ballX - sin(a)*robot.ballY,
                    robot.fieldY + sin(a)*robot.ballX + cos(a)*robot.ballY
                );
                
                ballTarget = Point(robot.ballTargetX, robot.ballTargetY);
            }
        }
    }
    
    if (handler == -1) {
        // std::cout << "Nobody sees the ball, searching!" << std::endl;
        // No one is seeing the ball, ordering all to search
        for (auto &entry : robots) {
            auto &robot = entry.second;
            info.order[robot.id-1] = rhoban_team_play::CaptainOrder::SearchBall;
        }
    } else {
        // Grabing robots that should be placed (all excepted handler and goal)
        std::vector<int> otherIds;
        for (auto &entry : robots) {
            auto &robot = entry.second;
            
            if (robot.id == handler) {
                // std::cout << "Handler is " << handler << std::endl;
                info.order[robot.id-1] = rhoban_team_play::CaptainOrder::HandleBall;
            } else {
                if (robot.state != rhoban_team_play::TeamPlayState::GoalKeeping) {
                    info.order[robot.id-1] = rhoban_team_play::CaptainOrder::Place;
                    for (int n=0; n<3; n++) {
                        info.robotTarget[robot.id-1][n] = 0;
                    }
                    otherIds.push_back(robot.id);
                }
            }
        }
        
        // Building targets
        auto targets = getTargetPositions(ball, ballTarget);
        
        // Optimizing the placing
        // std::cout << "Captain: There is " << otherIds.size() << " to place" << std::endl;
        
        auto solution = PlacementOptimizer::optimize(otherIds, targets, 
            [this](PlacementOptimizer::Solution solution) -> float {
                float score = 0;
                float ratio = 0;
                for (auto &robotTarget : solution.robotTarget) {
                    auto robot = robots[robotTarget.first];
                    float walkLength = (Point(robot.fieldX, robot.fieldY) - robotTarget.second.position).getLength();
                    ratio += robotTarget.second.data;
                    score += walkLength;
                }
                
                ratio /= solution.robotTarget.size();
                score += fabs(ratio-aggressivity)*1000;
                
                return score;
        });
        setSolution(solution);
    }
}

void CaptainService::compute()
{
    auto referee = getServices()->referee;
    auto teamPlay = getServices()->teamPlay;
    
    // First collecting robots that are able to play
    robots.clear();
    robotIds.clear();
    
    for (auto &entry : teamPlay->allInfo()) {
        auto info = entry.second;
        if (!info.isOutdated() && // Info should not be updated
        !referee->isPenalized(info.id) &&  // Robot should not be penalized
        info.state != rhoban_team_play::Unknown && // It should be playing
        info.fieldOk &&  // It knows where it is
        info.fieldX == info.fieldX && // The values are not NaNs
        info.fieldY == info.fieldY &&
        info.fieldYaw == info.fieldYaw 
    ) {
            robots[info.id] = info;
            robotIds.push_back(info.id);
        }
    }

    updateCommonBall();
    
    if (referee->isPlacingPhase()) {
        computeBasePositions();
    } else {
        computePlayingPositions();
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
    
    // Check for incoming captain messages
    while (_broadcaster->checkMessage((unsigned char*)&receive, len)) {
        if (len != sizeof(receive)) {
            // std::cout << "ERROR: CaptainService: invalid message" << std::endl;
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
        bind.pull();
        IAmCaptain = amICaptain();
        captainId = findCaptainId();
        lastTick = rhoban_utils::TimeStamp::now();
        // Updating
        if (teamPlay->isEnabled()) {
            if (IAmCaptain) {
                mutex.lock();
                compute();   
                mutex.unlock();
                     
                // Sending captain packet
                if (share) {
                    _broadcaster->broadcastMessage((unsigned char*)&info, sizeof(info));
                }
            }
        } else {
            compute();
        }
        bind.push();
        
        // Sleepint if needed to fit the given frequency
        auto now = rhoban_utils::TimeStamp::now();
        double elapsed = diffSec(lastTick, now);
        double toSleep = (1.0/CAPTAIN_FREQUENCY) - elapsed;
        if (toSleep > 0) {
            usleep(round(toSleep*1000000));
        }
    }    
}
