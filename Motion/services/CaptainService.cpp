#include <rhoban_utils/timing/time_stamp.h>
#include <services/TeamPlayService.h>
#include <strategy/PlacementOptimizer.h>
#include <services/RefereeService.h>
#include <robocup_referee/constants.h>
#include <unistd.h>
#include "CaptainService.h"

#include <rhoban_geometry/point_cluster.h>
#include <rhoban_team_play/team_play.h>
#include <rhoban_utils/logging/logger.h>
#include <rhoban_utils/util.h>

#include <hl_communication/robot_msg_utils.h>
#include <hl_communication/perception.pb.h>

using namespace hl_communication;
using namespace rhoban_utils;
using namespace robocup_referee;
using namespace rhoban_geometry;
using namespace rhoban_team_play;

static Logger logger("CaptainService");

/**
 * Helper to bound a position avoiding the penalty area
 */
static void boundPosition(Point& point)
{
  double xMin = -Constants::field.field_length / 2 + Constants::field.goal_area_length + 0.5;
  double xMax = Constants::field.field_length / 2 - Constants::field.goal_area_length;
  double yMax = Constants::field.goal_area_width / 2;

  if (point.x > xMax)
  {
    point.x = xMax;
    if (point.y > yMax)
      point.y = yMax;
    if (point.y < -yMax)
      point.y = -yMax;
  }
  if (point.x < xMin)
  {
    point.x = xMin;
    if (point.y > yMax)
      point.y = yMax;
    if (point.y < -yMax)
      point.y = -yMax;
  }
}

std::string CaptainService::Config::getClassName() const
{
  return "CaptainConfig";
}

void CaptainService::Config::fromJson(const Json::Value& json_value, const std::string& dir_name)
{
  basePositions.clear();
  if (json_value.isMember("positions"))
  {
    for (auto position : json_value["positions"])
    {
      BasePosition basePosition;
      basePosition.targetPosition = Point(position["x"].asFloat(), position["y"].asFloat());
      basePosition.targetOrientation = position["orientation"].asFloat();
      basePosition.kickOff = position["kickOff"].asBool();

      if (position.isMember("mandatory"))
      {
        basePosition.mandatory = position["mandatory"].asBool();
      }
      else
      {
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

CaptainService::CaptainService() : bind("captain"), frequency(5.0), recentlyKicked(false)
{
  // Loading configuration file
  config.loadFile("captain.json");

  bind.bindNew("frequency", frequency, RhIO::Bind::PullOnly)
      ->defaultValue(frequency)
      ->comment("Frequency of the update of the strategies [Hz]");

  bind.bindNew("passPlacingRatio", passPlacingRatio, RhIO::Bind::PullOnly)
      ->defaultValue(0.75)
      ->comment("Ratio to the kick vector to place");

  bind.bindNew("passPlacingOffset", passPlacingOffset, RhIO::Bind::PullOnly)
      ->defaultValue(0.35)
      ->comment("Offset to kick vector to place [m]")
      ->persisted(true);

  bind.bindNew("perpendicularBallDistance", perpendicularBallDistance, RhIO::Bind::PullOnly)
      ->defaultValue(1)
      ->comment("Distance [m] on the perpendicular to attack placement");

  bind.bindNew("placingBallDistance", placingBallDistance, RhIO::Bind::PullOnly)
      ->defaultValue(2.0)
      ->comment("Distance [m] to the ball for team play placement");

  bind.bindNew("avoidRadius", avoidRadius, RhIO::Bind::PullOnly)
      ->defaultValue(1.0)
      ->comment("Radius [m] to avoid colliding the ball while placing");

  bind.bindNew("minAggressivity", minAggressivity, RhIO::Bind::PullOnly)
      ->defaultValue(0.5)
      ->minimum(0.0)
      ->maximum(1.0)
      ->comment("Is the placing aggressive ore defensive?");

  bind.bindNew("maxAggressivity", maxAggressivity, RhIO::Bind::PullOnly)
      ->defaultValue(0.5)
      ->minimum(0.0)
      ->maximum(1.0)
      ->comment("Is the placing aggressive ore defensive?");

  bind.bindNew("aggressivity", aggressivity, RhIO::Bind::PushOnly)->comment("Computed aggressivity");

  bind.bindNew("captainId", captainId, RhIO::Bind::PushOnly);
  bind.bindNew("IAmCaptain", IAmCaptain, RhIO::Bind::PushOnly);

  bind.bindNew("handler", handler, RhIO::Bind::PushOnly)
      ->defaultValue(-1)
      ->comment("Id of the robot handling the ball");
  bind.bindNew("handlerChangeCost", handlerChangeCost, RhIO::Bind::PullOnly)
      ->defaultValue(3)
      ->comment("The cost of changing of handler [s]");
  bind.bindNew("noViewCost", noViewCost, RhIO::Bind::PullOnly)
      ->defaultValue(5)
      ->comment("The cost of not seeing the ball for a robot [s]");
  bind.bindNew("wrongBallCost", wrongBallCost, RhIO::Bind::PullOnly)
      ->defaultValue(20)
      ->comment("The cost of seeing a ball against consensus for handler selection [s]");

  bind.bindNew("commonBallTol", commonBallTol, RhIO::Bind::PullOnly)
      ->defaultValue(2.0)
      ->comment("Distance [m] for merging ball candidates");
  bind.bindNew("kickMemoryDuration", kickMemoryDuration, RhIO::Bind::PullOnly)
      ->defaultValue(1.0)
      ->comment("Time during which a kick is considered as recent [s]");

  bind.bindNew("oppToMateMinDist", oppToMateMinDist, RhIO::Bind::PullOnly)
      ->defaultValue(1.5)
      ->comment("Minimal distance from mates to consider opponents [m] ");

  bind.bindNew("oppMergeTol", oppMergeTol, RhIO::Bind::PullOnly)
      ->defaultValue(2.0)
      ->comment("Maximal distance to merge opponents [m] ");

  bind.bindNew("commonBallNbRobots", commonBallStrength, RhIO::Bind::PushOnly);
  bind.bindNew("commonBallX", commonBall.x, RhIO::Bind::PushOnly);
  bind.bindNew("commonBallY", commonBall.y, RhIO::Bind::PushOnly);

  bind.bindNew("recentlyKicked", recentlyKicked, RhIO::Bind::PushOnly);
}

CaptainService::~CaptainService()
{
  if (captainThread)
  {
    running = false;
    captainThread->join();
  }
}

int CaptainService::findCaptainId()
{
  auto referee = getServices()->referee;
  auto teamPlay = getServices()->teamPlay;
  auto& info = teamPlay->allInfo();
  share = true;

  for (const auto& entry : info)
  {
    const RobotMsg& robotInfo = entry.second;
    Action action = robotInfo.intention().action_planned();
    int robot_id = robotInfo.robot_id().robot_id();
    bool outdated = isOutdated(robotInfo);
    bool undefined_action = action == Action::UNDEFINED;
    bool penalized = referee->isPenalized(robot_id);
    // logger.log("RobotInfo: outdated:%d action:%d penalized:%d", outdated, undefined_action, penalized);
    if (!outdated && !undefined_action && !penalized)
    {
      return robot_id;
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

Captain CaptainService::getStatus()
{
  std::lock_guard<std::mutex> lock(mutex);
  return Captain(status);
}

StrategyOrder CaptainService::getMyOrder()
{
  std::lock_guard<std::mutex> lock(mutex);
  uint32_t my_id = getServices()->teamPlay->myId();
  for (const StrategyOrder& order : status.orders())
  {
    if (order.robot_id() == my_id)
    {
      return order;
    }
  }
  logger.warning("Cannot find an order for robot: %d", my_id);
  return StrategyOrder();
}

bool CaptainService::importCommonBall(rhoban_geometry::Point* dst)
{
  std::lock_guard<std::mutex> lock(mutex);
  if (!status.has_ball())
    return false;
  dst->x = status.ball().position().x();
  dst->y = status.ball().position().y();
}

void CaptainService::setStatus(const hl_communication::Captain& new_status)
{
  std::lock_guard<std::mutex> lock(mutex);
  status = new_status;
}

void CaptainService::setSolution(PlacementOptimizer::Solution solution)
{
  for (auto& robotTarget : solution.robotTarget)
  {
    int player_id = robotTarget.first;
    // Searching if an order for the given robot already exists
    StrategyOrder* order = nullptr;
    for (int idx = 0; idx < status.orders_size(); idx++)
    {
      int order_id = status.orders(idx).robot_id();
      if (order_id == player_id)
      {
        order = status.mutable_orders(idx);
      }
    }
    if (order == nullptr)
    {
      order = status.add_orders();
      order->set_robot_id(player_id);
    }
    // Updating order
    auto& target = robotTarget.second;
    PoseDistribution* target_pose = order->mutable_target_pose();
    target_pose->mutable_position()->set_x(target.position.x);
    target_pose->mutable_position()->set_y(target.position.y);
    target_pose->mutable_dir()->set_mean(target.orientation);
    order->set_action(Action::POSITIONING);
  }
}

void CaptainService::updateCommonBall()
{
  // Gather balls which are currently seen by other robots
  std::vector<Point> balls;
  bool tmpKicked = false;
  for (const auto& robot_entry : robots)
  {
    const RobotMsg& info = robot_entry.second;
    PerceptionExtra perception_extra = extractPerceptionExtra(info.perception());
    MiscExtra misc_extra = extractMiscExtra(info);

    bool ball_valid = perception_extra.ball().valid();
    bool field_valid = perception_extra.field().valid();
    bool robot_kicked = misc_extra.time_since_last_kick() < kickMemoryDuration;
    // logger.log("common_ball update: ball_valid:%d, field_valid:%d, robot_kicked:%d", ball_valid, field_valid,
    //           robot_kicked);
    if (ball_valid && field_valid && !robot_kicked)
    {
      PositionDistribution ball_in_field =
          fieldFromSelf(info.perception().self_in_field(0).pose(), info.perception().ball_in_self());
      balls.push_back(Point(ball_in_field.x(), ball_in_field.y()));
    }
    if (robot_kicked)
    {
      if (!recentlyKicked)
      {
        logger.log("MATEKICK: a robot of the team just kicked");
      }
      tmpKicked = true;
    }
  }

  recentlyKicked = tmpKicked;

  if (balls.size() == 0)
  {
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
  double max_dist = 15.0;  // [m]
  double best_cluster_score = 0;
  int best_cluster_id = -1;

  for (size_t cluster_id = 0; cluster_id < ball_clusters.size(); cluster_id++)
  {
    const PointCluster& cluster = ball_clusters[cluster_id];
    double cluster_score = cluster.size() * max_dist;
    if (last_common_ball.nb_votes() > 0)
    {
      Point old_pos(last_common_ball.position().x(), last_common_ball.position().y());
      if (!recentlyKicked)
      {
        cluster_score -= cluster.getAverage().getDist(old_pos);
      }
    }
    if (cluster_score > best_cluster_score)
    {
      best_cluster_score = cluster_score;
      best_cluster_id = cluster_id;
    }
  }

  // Explicit error in case someone changes scoring or conditions
  if (best_cluster_id == -1)
  {
    throw std::logic_error(DEBUG_INFO + "no best cluster found, should never happen");
  }

  const PointCluster& best_cluster = ball_clusters[best_cluster_id];
  Point common_pos = best_cluster.getAverage();
  hl_communication::CommonBall* common_ball = status.mutable_ball();
  common_ball->set_nb_votes(best_cluster.size());
  common_ball->mutable_position()->set_x(common_pos.x);
  common_ball->mutable_position()->set_y(common_pos.y);
}

void CaptainService::updateCommonOpponents()
{
  std::vector<Point> obstacles;
  std::vector<Point> mates;
  // Gathering all published obstacles and mates positions
  for (const auto& robot_entry : robots)
  {
    const RobotMsg& info = robot_entry.second;
    const PoseDistribution mate_pose = info.perception().self_in_field(0).pose();
    const PositionDistribution& mate_pos = mate_pose.position();
    mates.push_back(Point(mate_pos.x(), mate_pos.y()));
    for (const Perception::WeightedRobotPose& robot : info.perception().robots())
    {
      const PositionDistribution& pos = fieldFromSelf(mate_pose, robot.robot().robot_in_self().position());
      obstacles.push_back(Point(pos.x(), pos.y()));
    }
  }
  // Creating clusters
  std::vector<PointCluster> opponent_clusters;
  for (const Point& obs : obstacles)
  {
    // Do not consider obstacles near mates (they are considered as mates)
    bool near_mate = false;
    for (const auto& mate : mates)
    {
      if (obs.getDist(mate) < oppToMateMinDist)
      {
        near_mate = true;
        break;
      }
    }
    if (near_mate)
      continue;
    // Now we can add the point to existing clusters
    addToClusters(obs, opponent_clusters, oppMergeTol);
  }
  // List of opponents ordered by number of elements
  for (const PointCluster& c : opponent_clusters)
  {
    hl_communication::CommonOpponent* opponent = status.add_opponents();
    opponent->set_nb_votes(c.size());
    opponent->mutable_pose()->mutable_position()->set_x(c.getAverage().x);
    opponent->mutable_pose()->mutable_position()->set_y(c.getAverage().y);
  }
}

void CaptainService::computeBasePositions()
{
  auto referee = getServices()->referee;
  bool kickOff = referee->myTeamKickOff() && !referee->isDroppedBall();

  // List the possible targets (filters the base positions according
  // to the kickOff flag)
  std::vector<PlacementOptimizer::Target> targets;
  for (auto& basePosition : config.basePositions)
  {
    if (basePosition.kickOff == kickOff)
    {
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
  for (auto& id : robotIds)
  {
    const RobotMsg& robot = robots[id];
    if (robot.team_play().role() == Role::GOALIE)
    {
      goalId = id;
    }
    else
    {
      noGoalIds.push_back(id);
    }
  }

  // Finding the best solution
  auto solution =
      PlacementOptimizer::optimize(noGoalIds, targets, [this](PlacementOptimizer::Solution solution) -> float {
        float score = 0;
        for (auto& robotTarget : solution.robotTarget)
        {
          auto robot = robots[robotTarget.first];
          const PositionDistribution& robot_pos = robot.perception().self_in_field(0).pose().position();
          float walkLength = (Point(robot_pos.x(), robot_pos.y()) - robotTarget.second.position).getLength();
          if (robotTarget.second.mandatory)
          {
            walkLength *= 100;
          }
          score += walkLength;
        }
        return score;
      });
  setSolution(solution);

  // Sending the goalie in the goals
  if (goalId)
  {
    StrategyOrder* goalie_order = status.add_orders();
    goalie_order->set_robot_id(goalId);
    goalie_order->set_action(Action::POSITIONING);
    PoseDistribution* goalie_target = goalie_order->mutable_target_pose();
    double dist_to_goal = 0.25;  // Offset from goal line in [m]
    goalie_target->mutable_position()->set_x(-Constants::field.field_length / 2 + dist_to_goal);
    goalie_target->mutable_position()->set_y(0);
    goalie_target->mutable_dir()->set_mean(0);
  }
}

std::vector<PlacementOptimizer::Target> CaptainService::getTargetPositions(Point ball, Point ballTarget)
{
  std::vector<PlacementOptimizer::Target> targets;
  std::vector<PlacementOptimizer::Target> finalTargets;

  Point corner;
  Point vect = (ballTarget - ball) * passPlacingRatio - (ballTarget - ball).normalize() * passPlacingOffset;
  Point nVect = vect.perpendicular().normalize(perpendicularBallDistance);
  bool backward = ballTarget.x - 0.1 <= ball.x;
  Point ourGoalCenter(-Constants::field.field_length / 2, 0);
  auto oppositeCorner = corner;
  oppositeCorner.y *= -1;

  {
    PlacementOptimizer::Target target;

    corner = Point(Constants::field.field_length / 2, Constants::field.goal_width / 2 + 0.25);

    if (backward)
    {
      target.position = ball + (ballTarget - ball) * 1.15;
    }
    else
    {
      target.position = ball + vect + nVect;
    }
    target.data = 1;
    target.orientation = (ballTarget - target.position).getTheta().getSignedValue();
    targets.push_back(target);
  }
  {
    PlacementOptimizer::Target target;
    corner = Point(Constants::field.field_length / 2, -Constants::field.goal_width / 2 - 0.25);

    if (backward)
    {
      auto tmp1 = ball + vect + nVect;
      auto tmp2 = ball + vect + nVect;
      if (tmp1.x < tmp2.x)
        target.position = tmp1;
      else
        target.position = tmp2;
    }
    else
    {
      target.position = ball + vect - nVect;
    }
    target.data = 1;
    target.orientation = (ballTarget - target.position).getTheta().getSignedValue();
    targets.push_back(target);
  }

  {
    PlacementOptimizer::Target target;
    auto ballToGoal = (ourGoalCenter - ball);
    auto normal = ballToGoal.perpendicular().normalize(perpendicularBallDistance);
    target.position = ball + ballToGoal.normalize(placingBallDistance) + normal;

    target.data = 0;
    target.orientation = Angle::weightedAverage((ball - target.position).getTheta(), 0.5,
                                                (oppositeCorner - target.position).getTheta(), 0.5)
                             .getSignedValue();
    targets.push_back(target);
  }

  {
    PlacementOptimizer::Target target;
    auto ballToGoal = (ourGoalCenter - ball);
    auto normal = ballToGoal.perpendicular().normalize(perpendicularBallDistance);
    target.position = ball + ballToGoal.normalize(placingBallDistance) - normal;

    target.data = 0;
    target.orientation = Angle::weightedAverage((ball - target.position).getTheta(), 0.5,
                                                (oppositeCorner - target.position).getTheta(), 0.5)
                             .getSignedValue();
    targets.push_back(target);
  }

  for (auto& target : targets)
  {
    // Avoiding penalty areas
    boundPosition(target.position);

    // Avoiding being too close to the ball
    if ((target.position - ball).getLength() < avoidRadius)
    {
      continue;
    }

    // Target orientation
    finalTargets.push_back(target);
  }

  return finalTargets;
}

void CaptainService::computePlayingPositions()
{
  if (!status.has_ball())
  {
    logger.log("Nobody sees the ball, order all robots to search it!");
    // No one is seeing the ball, ordering all to search
    for (auto& entry : robots)
    {
      const RobotMsg& robot = entry.second;
      int robot_id = robot.robot_id().robot_id();
      StrategyOrder* order = status.add_orders();
      order->set_robot_id(robot_id);
      order->set_action(Action::SEARCHING_BALL);
    }
    handler = -1;
    return;
  }

  int newHandler = -1;
  double smallestTime = std::numeric_limits<double>::max();

  // XXX: We should check if the goal keeper is handling the ball, because in
  // this case we should not give the handle ball order to another robot (might
  // be 2 simultaneous attackers)
  for (auto& entry : robots)
  {
    const RobotMsg& robot_msg = entry.second;
    const PerceptionExtra& perception_extra = extractPerceptionExtra(robot_msg.perception());
    const PoseDistribution& self_in_field = robot_msg.perception().self_in_field(0).pose();
    double robotSpeed = 0.2;  // [m/s], should be extracted elsewhere
    int robot_id = robot_msg.robot_id().robot_id();
    // Computing cost
    double cost = 0;
    double dist = 0;
    if (handler != robot_id)
    {
      cost += handlerChangeCost;
    }
    rhoban_geometry::Point common_ball(status.ball().position().x(), status.ball().position().y());
    // If robot is able to see ball, use his own ball for time estimation
    // Otherwise, use common ball and add a penalty
    if (perception_extra.ball().valid())
    {
      const PositionDistribution& ball_in_self = robot_msg.perception().ball_in_self();
      dist = sqrt(pow(ball_in_self.x(), 2) + pow(ball_in_self.y(), 2));

      const PositionDistribution& robot_ball_dist = getBallInField(robot_msg);
      Point robot_ball(robot_ball_dist.x(), robot_ball_dist.y());
      if (robot_ball.getDist(common_ball) > commonBallTol)
      {
        cost += wrongBallCost;
      }
    }
    else
    {
      const PositionDistribution& robot_pos = self_in_field.position();
      double dx = common_ball.x - robot_pos.x();
      double dy = common_ball.y - robot_pos.y();
      dist = sqrt(pow(dx, 2) + pow(dy, 2));
      cost += noViewCost;
    }
    cost += dist / robotSpeed;

    if (cost < smallestTime)
    {
      newHandler = robot_id;
      smallestTime = cost;
    }

    logger.log("Cost for %d: %f (current handler %d)", robot_id, cost, handler);
  }
  // Grabing robots that should be placed (all excepted newHandler and goal)
  std::vector<int> otherIds;
  for (auto& entry : robots)
  {
    const RobotMsg& robot = entry.second;
    int robot_id = robot.robot_id().robot_id();

    if (robot_id == newHandler)
    {
      StrategyOrder* order = status.add_orders();
      order->set_robot_id(robot_id);
      order->set_action(Action::GOING_TO_KICK);
    }
    else if (robot.team_play().role() != Role::GOALIE)
    {
      // Orders will be updated based on the PlacementOptimizer
      otherIds.push_back(robot_id);
    }
  }

  // Building targets
  const RobotMsg& kicker = robots[newHandler];
  Point common_ball(status.ball().position().x(), status.ball().position().y());
  const PositionDistribution& kickTarget = kicker.intention().kick().target();
  Point ballTarget = Point(kickTarget.x(), kickTarget.y());
  auto targets = getTargetPositions(common_ball, ballTarget);

  // Optimizing the placing
  // std::cout << "Captain: There is " << otherIds.size() << " to place" << std::endl;

  // Adaptative aggressivity
  double ballRatio = (common_ball.x + Constants::field.field_length / 2) / Constants::field.field_length;
  aggressivity = minAggressivity + ballRatio * (maxAggressivity - minAggressivity);

  auto solution =
      PlacementOptimizer::optimize(otherIds, targets, [this](PlacementOptimizer::Solution solution) -> float {
        float score = 0;
        float ratio = 0;
        for (auto& robotTarget : solution.robotTarget)
        {
          const RobotMsg& robot = robots[robotTarget.first];
          const PositionDistribution& robot_pos = robot.perception().self_in_field(0).pose().position();
          float walkLength = (Point(robot_pos.x(), robot_pos.y()) - robotTarget.second.position).getLength();
          ratio += robotTarget.second.data;
          score += walkLength;
        }

        ratio /= solution.robotTarget.size();
        score += fabs(ratio - aggressivity) * 1000;

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

  for (const auto& entry : teamPlay->allInfo())
  {
    const RobotMsg& info = entry.second;
    Action action = info.intention().action_planned();
    int robot_id = info.robot_id().robot_id();
    PerceptionExtra extra = extractPerceptionExtra(info.perception());
    if (!isOutdated(info) &&                                          // Info should not be updated
        !referee->isPenalized(robot_id) &&                            // Robot should not be penalized
        action != Action::UNDEFINED && action != Action::INACTIVE &&  // It should be playing
        extra.field().valid()                                         // It knows where it is
    )
    {
      robots[robot_id] = info;
      robotIds.push_back(robot_id);
    }
  }

  last_common_ball.Clear();
  if (status.has_ball())
  {
    last_common_ball = status.ball();
  }
  status.Clear();

  updateCommonBall();
  updateCommonOpponents();

  if (referee->isPlacingPhase())
  {
    computeBasePositions();
  }
  else
  {
    computePlayingPositions();
  }
}

bool CaptainService::tick(double elapsed)
{
  if (!captainThread)
  {
    // Running the captain thread
    running = true;
    captainThread.reset(new std::thread([this] { this->execThread(); }));
  }
  return true;
}

void CaptainService::execThread()
{
  bind.pull();
  auto lastTick = rhoban_utils::TimeStamp::now();
  auto teamPlay = getServices()->teamPlay;

  while (running)
  {
    bind.pull();
    IAmCaptain = amICaptain();
    captainId = findCaptainId();
    lastTick = rhoban_utils::TimeStamp::now();
    // Updating
    mutex.lock();
    if (!teamPlay->isEnabled() || IAmCaptain)
    {
      compute();
    }
    mutex.unlock();
    bind.push();

    // Sleepint if needed to fit the given frequency
    auto now = rhoban_utils::TimeStamp::now();
    double elapsed = diffSec(lastTick, now);
    double toSleep = (1.0 / frequency) - elapsed;
    if (toSleep > 0)
    {
      usleep(round(toSleep * 1000000));
    }
  }
}
