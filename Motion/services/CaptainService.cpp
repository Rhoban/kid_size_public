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

using rhoban_team_play::CommonOpponent;

static Logger logger("CaptainService");

/**
 * Helper to bound a position avoiding the penalty area
 */
static void boundPosition(Point &point)
{
    double xMin = -Constants::field.fieldLength/2 + Constants::field.goalAreaLength + 0.5;
    double xMax = Constants::field.fieldLength/2 - Constants::field.goalAreaLength;
    double yMax = Constants::field.goalAreaWidth/2;

    if (point.x > xMax) {
        point.x = xMax;
        if (point.y > yMax) point.y = yMax;
        if (point.y < -yMax) point.y = -yMax;
    }
    if (point.x < xMin) {
        point.x = xMin;
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
  : bind("captain"), captainThread(NULL),
    recentlyKicked(false)
{
    // Loading configuration file
    config.loadFile("captain.json");
        
    bind.bindNew("passPlacingRatio", passPlacingRatio, RhIO::Bind::PullOnly)
        ->defaultValue(0.75)->comment("Ratio to the kick vector to place");
    
    bind.bindNew("passPlacingOffset", passPlacingOffset, RhIO::Bind::PullOnly)
        ->defaultValue(0.35)->comment("Offset to kick vector to place [m]")->persisted(true);
        
    bind.bindNew("perpendicularBallDistance", perpendicularBallDistance, RhIO::Bind::PullOnly)
        ->defaultValue(1)->comment("Distance [m] on the perpendicular to attack placement");
        
    bind.bindNew("placingBallDistance", placingBallDistance, RhIO::Bind::PullOnly)
        ->defaultValue(2.0)->comment("Distance [m] to the ball for team play placement");
        
    bind.bindNew("avoidRadius", avoidRadius, RhIO::Bind::PullOnly)
        ->defaultValue(1.0)->comment("Radius [m] to avoid colliding the ball while placing");
        
    bind.bindNew("minAggressivity", minAggressivity, RhIO::Bind::PullOnly)
        ->defaultValue(0.5)->minimum(0.0)->maximum(1.0)
        ->comment("Is the placing aggressive ore defensive?");
    
    bind.bindNew("maxAggressivity", maxAggressivity, RhIO::Bind::PullOnly)
        ->defaultValue(0.5)->minimum(0.0)->maximum(1.0)
        ->comment("Is the placing aggressive ore defensive?");
        
    bind.bindNew("aggressivity", aggressivity, RhIO::Bind::PushOnly)
        ->comment("Computed aggressivity");
    
    bind.bindNew("captainId", captainId, RhIO::Bind::PushOnly);
    bind.bindNew("IAmCaptain", IAmCaptain, RhIO::Bind::PushOnly);

    bind.bindNew("handler", handler, RhIO::Bind::PushOnly)
      ->defaultValue(-1)->comment("Id of the robot handling the ball");
    bind.bindNew("handlerChangeCost", handlerChangeCost, RhIO::Bind::PullOnly)
      ->defaultValue(3)->comment("The cost of changing of handler [s]");
    bind.bindNew("noViewCost", noViewCost, RhIO::Bind::PullOnly)
      ->defaultValue(5)->comment("The cost of not seeing the ball for a robot [s]");
    bind.bindNew("wrongBallCost", wrongBallCost, RhIO::Bind::PullOnly)
      ->defaultValue(20)
      ->comment("The cost of seeing a ball against consensus for handler selection [s]");

    bind.bindNew("commonBallTol", commonBallTol, RhIO::Bind::PullOnly)
      ->defaultValue(2.0)->comment("Distance [m] for merging ball candidates");
    bind.bindNew("kickMemoryDuration", kickMemoryDuration, RhIO::Bind::PullOnly)
      ->defaultValue(1.0)
      ->comment("Time during which a kick is considered as recent [s]");

    bind.bindNew("oppToMateMinDist", oppToMateMinDist, RhIO::Bind::PullOnly)
      ->defaultValue(1.5)->comment("Minimal distance from mates to consider opponents [m] ");

    bind.bindNew("oppMergeTol", oppMergeTol, RhIO::Bind::PullOnly)
      ->defaultValue(2.0)->comment("Maximal distance to merge opponents [m] ");

    bind.bindNew("commonBallNbRobots", info.common_ball.nbRobots, RhIO::Bind::PushOnly);
    bind.bindNew("commonBallX", info.common_ball.x, RhIO::Bind::PushOnly);
    bind.bindNew("commonBallY", info.common_ball.y, RhIO::Bind::PushOnly);

    bind.bindNew("recentlyKicked", recentlyKicked, RhIO::Bind::PushOnly);
    
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
        instruction.ball = tmp.common_ball;
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
  std::vector<Point> balls;
  bool tmpKicked = false;
  for (const auto & robot_entry : robots) {
    const rhoban_team_play::TeamPlayInfo & info = robot_entry.second;

    if (info.ballOk && info.timeSinceLastKick > kickMemoryDuration) {
      balls.push_back(Point(info.getBallInField()));
    }
    if (info.timeSinceLastKick < kickMemoryDuration) {
      if (!recentlyKicked) {
        logger.log("MATEKICK: a robot of the team just kicked");
      }
      tmpKicked = true;
    }
  }

  recentlyKicked = tmpKicked;

  if (balls.size() == 0) {
    info.common_ball.x = 0;
    info.common_ball.y = 0;
    info.common_ball.nbRobots = 0;
    return;
  }

  // Place balls inside clusters
  // Disclaimer: order of balls can have an influence on cluster composition
  std::vector<PointCluster> ball_clusters = createClusters(balls, commonBallTol);

  // Choosing best cluster with the following criteria (by priority order)
  // 1. Size of the cluster
  // 2. Distance to last common ball if applicable
  //    (Clusters have same size and there are no recent kicks)
  // 3. First in list (if other criteria were not applicable)
  double max_dist = 15.0;// [m]
  double best_cluster_score = 0;
  int best_cluster_id = -1;

  for (size_t cluster_id = 0; cluster_id < ball_clusters.size(); cluster_id++) {
    const PointCluster & cluster = ball_clusters[cluster_id];
    double cluster_score = cluster.size() * max_dist;
    if (info.common_ball.nbRobots > 0) {
      Point old_pos(info.common_ball.x, info.common_ball.y);
      if (!recentlyKicked) {
        cluster_score -= cluster.getAverage().getDist(old_pos);
      }
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
}

void CaptainService::updateCommonOpponents() {
  std::vector<Point> obstacles;
  std::vector<Point> mates;
  // Gathering all published obstacles and mates positions
  for (const auto & robot_entry : robots) {
    const rhoban_team_play::TeamPlayInfo & info = robot_entry.second;
    mates.push_back(Point(info.fieldX, info.fieldY));
    for (int k = 0; k < info.nbObstacles; k++) {
      obstacles.push_back(Point(info.obstacles[k][0], info.obstacles[k][1]));
    }
  }
  // Creating clusters
  std::vector<PointCluster> opponent_clusters;
  for (const Point & obs : obstacles) {
    // Do not consider obstacles near mates (they are considered as mates)
    bool near_mate = false;
    for (const auto & mate : mates) {
      if (obs.getDist(mate) < oppToMateMinDist) {
        near_mate = true;
        break;
      }
    }
    if (near_mate) continue;
    // Now we can add the point to existing clusters
    addToClusters(obs, opponent_clusters, oppMergeTol);
  }
  // List of opponents ordered by number of elements
  std::vector<CommonOpponent> ordered_opponents;
  for (const PointCluster & c : opponent_clusters) {
    CommonOpponent opp;
    opp.consensusStrength = c.size();
    opp.x = c.getAverage().x;
    opp.y = c.getAverage().y;
    ordered_opponents.push_back(opp);
  }
  std::sort(ordered_opponents.begin(), ordered_opponents.end(),
            [](const CommonOpponent & o1, const CommonOpponent & o2) {
              return o1.consensusStrength > o2.consensusStrength;
            });
  // Ensuring we publish a limited number of opponents
  int nb_shared_opponents = (int) ordered_opponents.size();
  if (nb_shared_opponents > MAX_OPPONENTS) {
    logger.log("Too many common opponents (%d), keeping only %d",
               nb_shared_opponents, MAX_OPPONENTS);
    nb_shared_opponents = MAX_OPPONENTS;
  }
  info.nb_opponents = nb_shared_opponents;
  for (int k = 0; k < nb_shared_opponents; k++) {
    info.common_opponents[k] = ordered_opponents[k];
  }
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
    
    // Collecting non-goalie robots
    std::vector<int> noGoalIds;
    int goalId = 0;
    for (auto &id : robotIds) {
        auto &robot = robots[id];
        if (!robot.goalKeeper) {
            noGoalIds.push_back(id);
        } else {
            goalId = id;
        }
    }
    
    // Finding the best solution
    auto solution = PlacementOptimizer::optimize(noGoalIds, targets, 
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
    
    // Sending the goalie in the goals
    if (goalId) {
        info.robotTarget[goalId-1][0] = -Constants::field.fieldLength/2 + 0.25;
        info.robotTarget[goalId-1][1] = 0;
        info.robotTarget[goalId-1][2] = 0;
        info.order[goalId-1] = rhoban_team_play::CaptainOrder::Place;
    }
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
    Point ourGoalCenter(-Constants::field.fieldLength/2, 0);
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
        auto ballToGoal = (ourGoalCenter-ball);
        auto normal = ballToGoal.perpendicular().normalize(perpendicularBallDistance);
        target.position = ball+ballToGoal.normalize(placingBallDistance)+normal;
        
        target.data = 0;
        target.orientation = Angle::weightedAverage((ball-target.position).getTheta(), 0.5, 
            (oppositeCorner-target.position).getTheta(), 0.5).getSignedValue();
        targets.push_back(target);
    }
    
    {
        PlacementOptimizer::Target target;
        auto ballToGoal = (ourGoalCenter-ball);
        auto normal = ballToGoal.perpendicular().normalize(perpendicularBallDistance);
        target.position = ball+ballToGoal.normalize(placingBallDistance)-normal;
        
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
    if (info.common_ball.nbRobots == 0) {
        // std::cout << "Nobody sees the ball, searching!" << std::endl;
        // No one is seeing the ball, ordering all to search
        for (auto &entry : robots) {
            auto &robot = entry.second;
            info.order[robot.id-1] = rhoban_team_play::CaptainOrder::SearchBall;
        }
        handler = -1;
        return;
    }

    int newHandler = -1;
    double smallestTime = std::numeric_limits<double>::max();
    
    // XXX: We should check if the goal keeper is handling the ball, because in
    // this case we should not give the handle ball order to another robot (might
    // be 2 simultaneous attackers)
    for (auto &entry : robots) {
        auto &robot = entry.second;
        double robotSpeed = 0.2;// [m/s], should be extracted elsewhere
        // Computing cost
        double cost = 0;
        double dist = 0;
        if (handler != robot.id) {
          cost += handlerChangeCost;
        }
        // If robot is able to see ball, use his own ball for time estimation
        // Otherwise, use common ball and add a penalty
        if (robot.ballOk) {
          dist = sqrt(pow(robot.ballX, 2) + pow(robot.ballY, 2));

          Point robot_ball = robot.getBallInField();
          Point common_ball(info.common_ball.x, info.common_ball.y);
          if (robot_ball.getDist(common_ball) > commonBallTol) {
            cost += wrongBallCost;
          }
        } else {
          double dx = info.common_ball.x - robot.fieldX;
          double dy = info.common_ball.y - robot.fieldY;
          dist = sqrt(pow(dx, 2) + pow(dy, 2));
          cost += noViewCost;
        }
        cost += dist / robotSpeed;
            
        if (cost < smallestTime) {
          newHandler = robot.id;
          smallestTime = cost;
        }

        logger.log("Cost for %d: %f (current handler %d)", robot.id, cost, handler);
    }
    // Grabing robots that should be placed (all excepted newHandler and goal)
    std::vector<int> otherIds;
    for (auto &entry : robots) {
      auto &robot = entry.second;
      
      if (robot.id == newHandler) {
        // std::cout << "NewHandler is " << newHandler << std::endl;
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
    const auto & kicker = robots[newHandler];
    Point ball(info.common_ball.x, info.common_ball.y);
    Point ballTarget = Point(kicker.ballTargetX, kicker.ballTargetY);
    auto targets = getTargetPositions(ball, ballTarget);
        
    // Optimizing the placing
    // std::cout << "Captain: There is " << otherIds.size() << " to place" << std::endl;
        
    // Adaptative aggressivity
    double ballRatio = (info.common_ball.x + Constants::field.fieldLength/2) / Constants::field.fieldLength;
    aggressivity = minAggressivity + ballRatio * (maxAggressivity - minAggressivity);
        
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
    handler = newHandler;
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
        info.state != rhoban_team_play::Inactive &&
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
    updateCommonOpponents();
    
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
        handler = info.getHandler();
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
