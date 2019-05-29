#include <iostream>
#include <stdexcept>
#include <rhoban_utils/timing/time_stamp.h>
#include <rhoban_utils/logging/logger.h>

#include "CaptainService.h"
#include "DecisionService.h"
#include "LocalisationService.h"
#include "ModelService.h"
#include "Services.h"
#include "RefereeService.h"
#include "StrategyService.h"
#include "TeamPlayService.h"
#include "moves/Move.h"
#include "moves/Placer.h"
#include "moves/Playing.h"
#include "moves/Robocup.h"

#include <hl_communication/robot_msg_utils.h>
#include <hl_communication/utils.h>

static rhoban_utils::Logger logger("teamplay_service");

using namespace hl_communication;
using namespace rhoban_utils;
using namespace rhoban_geometry;
using namespace rhoban_team_play;

TeamPlayService::TeamPlayService()
  : _bind(nullptr)
  , _selfInfo()
  , _allInfo()
  , _t(0.0)
  , _isEnabled(true)
  , _broadcastPeriod(0.2)
  , _maxObstacles(10)
  , _isFieldInverted(false)
  , last_team_id(-1)
{
  // Initialize RhiO
  _bind = new RhIO::Bind("teamplay");
  _bind->bindNew("enable", _isEnabled, RhIO::Bind::PullOnly)->defaultValue(true);
  _bind->bindNew("broadcastPeriod", _broadcastPeriod, RhIO::Bind::PullOnly)
      ->comment("UDP broadcast period in seconds")
      ->defaultValue(0.2);
  _bind->bindNew("maxObstacles", _maxObstacles, RhIO::Bind::PullOnly)
      ->comment("Maximal number of obstacles in a message")
      ->defaultValue(10);
  _bind->bindNew("isFieldInverted", _isFieldInverted, RhIO::Bind::PullOnly)
      ->comment("true if robot attacks left of the team area")
      ->defaultValue(false);
  _bind->bindFunc("team", "Display information about teamplay", &TeamPlayService::cmdTeam, *this);

  kickOffClearanceDist = 1.05;
  _bind->bindNew("kickOffClearanceDist", kickOffClearanceDist, RhIO::Bind::PullOnly)
      ->minimum(0.0)
      ->maximum(2.0)
      ->defaultValue(kickOffClearanceDist)
      ->comment("Distance to ball when kick off is given to opponent team");
  gameInterruptionClearanceDist = 0.7;
  _bind->bindNew("gameInterruptionClearanceDist", gameInterruptionClearanceDist, RhIO::Bind::PullOnly)
      ->minimum(0.0)
      ->maximum(2.0)
      ->defaultValue(gameInterruptionClearanceDist)
      ->comment("Distance to ball when a game interruption is awarded to opponent team");

  // Initialize UDP communication
  _t = 0;
}

TeamPlayService::~TeamPlayService()
{
  delete _bind;
}

int TeamPlayService::myId()
{
  return Helpers::getServices()->referee->id;
}

const RobotMsg& TeamPlayService::selfInfo() const
{
  return _selfInfo;
}

const std::map<int, RobotMsg>& TeamPlayService::allInfo() const
{
  return _allInfo;
}

bool TeamPlayService::tick(double elapsed)
{
  _bind->pull();

  // If team id is available and team id
  int teamId = getServices()->referee->teamId;
  if (teamId != last_team_id)
  {
    if (last_team_id != -1)
    {
      int protobuf_team_port = getDefaultTeamPort(last_team_id);
      logger.log("Ending teamplay service on port %d", protobuf_team_port);
      message_manager.reset(nullptr);
      _allInfo.clear();
    }
    int protobuf_team_port = getDefaultTeamPort(teamId);
    logger.log("Starting teamplay service on port %d", protobuf_team_port);
    message_manager.reset(new UDPMessageManager(protobuf_team_port, protobuf_team_port));
    last_team_id = teamId;
  }

  // Sending informations at fixed frequency
  if (_t >= _broadcastPeriod)
  {
    _t -= _broadcastPeriod;
    messageSend();
  }
  _t += elapsed;

  if (_isEnabled && message_manager)
  {
    // Receiving informations
    GameMsg received_msg;
    while (message_manager->receiveMessage(&received_msg))
    {
      // Assign reception timestamp
      if (received_msg.has_robot_msg())
      {
        received_msg.mutable_robot_msg()->set_time_stamp(getTimeStamp());
        processInfo(received_msg.robot_msg());
      }
      else
      {
        logger.log("Received a message which is not a robot message");
      }
    }
  }
  else
  {
    _allInfo.clear();
  }

  _bind->push();

  return true;
}

bool TeamPlayService::isEnabled()
{
  return _isEnabled;
}

void TeamPlayService::updateIdentifier(RobotMsg* msg)
{
  RobotIdentifier* identifier = msg->mutable_robot_id();
  identifier->set_robot_id(myId());
  identifier->set_team_id(getServices()->referee->teamId);
}

void TeamPlayService::updatePerception(RobotMsg* msg)
{
  // Retrieving information from services
  auto decision = getServices()->decision;
  auto loc = getServices()->localisation;
  Point ballPos = loc->getBallPosSelf();
  Point ballVel = loc->getBallSpeedSelf();
  // Ball position in self
  Perception* perception = msg->mutable_perception();
  PositionDistribution* ball_in_self = perception->mutable_ball_in_self();
  ball_in_self->set_x(ballPos.x);
  ball_in_self->set_y(ballPos.y);
  PositionDistribution* ball_velocity_in_self = perception->mutable_ball_velocity_in_self();
  ball_velocity_in_self->set_x(ballVel.x);
  ball_velocity_in_self->set_y(ballVel.y);
  // Pose estimation of the robot, currently, only one pose is used
  rhoban_geometry::Point fieldPos = loc->getFieldPos();
  WeightedPose* self_in_field = perception->add_self_in_field();
  self_in_field->set_probability(1.0);
  self_in_field->mutable_pose()->mutable_position()->set_x(fieldPos.x);
  self_in_field->mutable_pose()->mutable_position()->set_y(fieldPos.y);
  self_in_field->mutable_pose()->mutable_dir()->set_mean(loc->getFieldOrientation());
  // Adding obstacles to message
  const std::vector<Eigen::Vector3d>& opponents = loc->getOpponentsSelf();
  size_t nb_opponents = std::min(opponents.size(), (size_t)_maxObstacles);
  if (opponents.size() > (size_t)_maxObstacles)
  {
    logger.warning("Too many obstacles to broadcast");
  }
  for (size_t opp_idx = 0; opp_idx < nb_opponents; opp_idx++)
  {
    Perception::WeightedRobotPose* robot = perception->add_robots();
    robot->set_probability(1.0);
    PoseDistribution* robot_pose = robot->mutable_robot()->mutable_robot_in_self();
    // TODO: check if opponent robot is not in world_referential
    robot_pose->mutable_position()->set_x(opponents[opp_idx].x());
    robot_pose->mutable_position()->set_y(opponents[opp_idx].y());
  }

  // Custom information
  PerceptionExtra extra;
  BallQuality* extra_ball = extra.mutable_ball();
  extra_ball->set_quality(loc->ballQ);
  extra_ball->set_valid(decision->isBallQualityGood);
  FieldQuality* extra_field = extra.mutable_field();
  extra_field->set_quality(loc->fieldQ);
  extra_field->set_consistency(loc->fieldConsistency);
  extra_field->set_valid(decision->isFieldQualityGood);
  extra.SerializeToString(perception->mutable_free_field());
}

void TeamPlayService::updateIntention(RobotMsg* msg)
{
  Intention* intention = msg->mutable_intention();
  PlayingMove* playing = dynamic_cast<PlayingMove*>(getMoves()->getMove("playing"));
  Placer* placer = dynamic_cast<Placer*>(getMoves()->getMove("placer"));
  Robocup* robocup = dynamic_cast<Robocup*>(getMoves()->getMove("robocup"));
  if (!playing)
  {
    throw std::logic_error(DEBUG_INFO + "'playing' move is not of the right type");
  }
  if (!placer)
  {
    throw std::logic_error(DEBUG_INFO + "'placer' move is not of the right type");
  }
  // Updating Action
  Action action = Action::WAITING;
  bool robocup_running = robocup->isRunning();
  bool playing_running = playing->isRunning();
  bool approach_running = getMoves()->getMove("approach_potential")->isRunning();
  // During test sessions, playing and approach can run alone, in this case, we do not want to declare the robot as
  // inactive
  if (!playing_running && !approach_running && (!robocup_running || robocup->getStatus() != "placing"))
  {
    action = Action::INACTIVE;
  }
  else if (getMoves()->getMove("kick")->isRunning())
  {
    action = Action::KICKING;
  }
  else if (getMoves()->getMove("search")->isRunning())
  {
    action = Action::SEARCHING_BALL;
  }
  else if (placer->isRunning() && !placer->arrived)
  {
    action = Action::POSITIONING;
  }
  else if (playing->isRunning())
  {
    if (playing->getStatus() == "approach" || playing->getStatus() == "walkBall")
    {
      action = Action::GOING_TO_KICK;
    }
  }
  intention->set_action_planned(action);
  // Updating target position if appliable
  if (placer->isRunning())
  {
    Eigen::Vector2d waypoint_pos = placer->getLocalTarget();
    PositionDistribution* waypoint = intention->add_waypoints_in_field()->mutable_position();
    waypoint->set_x(waypoint_pos.x());
    waypoint->set_y(waypoint_pos.y());
    Eigen::Vector2d target_pos = placer->getTarget();
    PositionDistribution* target = intention->mutable_target_pose_in_field()->mutable_position();
    target->set_x(target_pos.x());
    target->set_y(target_pos.y());
  }
  // Updating kick intention
  LocalisationService* loc = getServices()->localisation;
  StrategyService* strategy = getServices()->strategy;
  if (strategy->getActiveKickController() != nullptr)
  {
    KickIntention* kick = intention->mutable_kick();
    rhoban_geometry::Point ball_in_field = loc->getBallPosField();
    Eigen::Vector2d kick_target = strategy->getKickTarget();
    kick->mutable_start()->set_x(ball_in_field.x);
    kick->mutable_start()->set_y(ball_in_field.y);
    kick->mutable_target()->set_x(kick_target.x());
    kick->mutable_target()->set_y(kick_target.y());
  }
}

void TeamPlayService::updateTeamPlay(RobotMsg* msg)
{
  Robocup* robocup = dynamic_cast<Robocup*>(getMoves()->getMove("robocup"));
  if (!robocup)
  {
    throw std::logic_error(DEBUG_INFO + " robocup type is wrong");
  }
  msg->mutable_team_play()->set_role(robocup->isGoalKeeper() ? Role::GOALIE : Role::UNSPECIFIED_ROLE);
}

void TeamPlayService::updateMiscExtra(RobotMsg* msg)
{
  StrategyService* strategy = getServices()->strategy;
  ModelService* model = getServices()->model;

  MiscExtra extra;
  extra.set_time_since_last_kick(strategy->getTimeSinceLastKick());
  extra.set_referee(getServices()->referee->getState().substr(0, 15));
  extra.set_robocup(getMoves()->getMove("robocup")->getStatus().substr(0, 10));
  extra.set_playing(getMoves()->getMove("playing")->getStatus().substr(0, 10));
  extra.set_search(getMoves()->getMove("search")->getStatus().substr(0, 10));
  extra.set_hardware_warnings(model->lowLevelState.substr(0, 30));
  extra.SerializeToString(msg->mutable_free_field());
}

void TeamPlayService::messageSend()
{
  _selfInfo.Clear();

  _selfInfo.set_time_stamp(getTimeStamp());
  _selfInfo.set_utc_time_stamp(getUTCTimeStamp());
  updateIdentifier(&_selfInfo);
  updatePerception(&_selfInfo);
  updateIntention(&_selfInfo);
  updateTeamPlay(&_selfInfo);
  updateMiscExtra(&_selfInfo);
  CaptainService* captain_service = Helpers::getServices()->captain;
  if (captain_service->amICaptain())
  {
    _selfInfo.mutable_captain()->CopyFrom(captain_service->getStatus());
  }

  // Convert selfInfo to Protobuf
  if (_isEnabled && message_manager)
  {
    hl_communication::GameMsg public_message;
    public_message.mutable_robot_msg()->CopyFrom(_selfInfo);
    if (_isFieldInverted)
    {
      invertField(public_message.mutable_robot_msg());
    }
    message_manager->sendMessage(&public_message);
  }
}

void TeamPlayService::processInfo(const RobotMsg& original_msg)
{
  if (!original_msg.has_robot_id())
  {
    logger.warning("Received a message without robot_id");
    return;
  }
  const RobotIdentifier& msg_identifier = original_msg.robot_id();
  int msg_team_id = msg_identifier.team_id();
  int msg_robot_id = msg_identifier.robot_id();
  if (msg_team_id != getServices()->referee->teamId)
  {
    logger.warning("Received a message from another team: %d", msg_team_id);
    return;
  }
  if (!original_msg.has_perception())
  {
    logger.warning("Received a message with no perception");
    return;
  }

  // Updates stored informations
  RobotMsg msg = original_msg;
  if (_isFieldInverted)
  {
    invertField(&msg);
  }
  _allInfo[msg_robot_id] = msg;
  // Updating captain data if message contains captain message
  if (msg.has_captain())
  {
    getServices()->captain->setStatus(msg.captain());
  }
  // Updating shared opponents based on message (duplication with captain updateCommonOpponents???)
  if (msg_robot_id != myId() && msg.has_perception() && msg.perception().self_in_field_size() > 0)
  {
    const Perception& perception = msg.perception();
    // WARNING: currently only first pos emitted by the sender is used
    const PoseDistribution& sender_in_field = perception.self_in_field(0).pose();
    // Update opponents position;
    std::vector<Eigen::Vector2d> opponents_seen;
    for (const Perception::WeightedRobotPose& estimation : perception.robots())
    {
      const PositionDistribution& robot_in_self = estimation.robot().robot_in_self().position();
      const PositionDistribution& opp = fieldFromSelf(sender_in_field, robot_in_self);
      opponents_seen.push_back(Eigen::Vector2d(opp.x(), opp.y()));
    }
    getServices()->localisation->updateSharedOpponents(msg_robot_id, opponents_seen);
  }
}

std::string TeamPlayService::cmdTeam()
{
  std::stringstream ss;
  ss << "There is " << _allInfo.size() << " players." << std::endl;
  if (myId() == 0)
  {
    ss << "WARNING: I am not a player, because my id is 0" << std::endl;
  }
  ss << std::endl;
  for (auto entry : _allInfo)
  {
    auto info = entry.second;
    PerceptionExtra perception_extra = extractPerceptionExtra(info.perception());
    MiscExtra misc_extra = extractMiscExtra(info);
    const PositionDistribution& ball = info.perception().ball_in_self();
    const PositionDistribution& field = info.perception().self_in_field(0).pose().position();
    ss << "Player #" << getRobotId(info);
    if (getRobotId(info) == myId())
    {
      ss << " (me)";
    }
    ss << " - last update: " << getAge(info) / 1000 << "ms" << std::endl;
    ss << "Action planned: " << action2str(info.intention().action_planned()) << std::endl;
    ss << "- Ball distance: " << getBallDistance(info) << ", X=" << ball.x() << " Y=" << ball.y()
       << ", quality: " << perception_extra.ball().quality() << std::endl;
    ss << "- Field X: " << field.x() << ", Y: " << field.y() << ", quality: " << perception_extra.field().quality()
       << std::endl;
    ss << "- State Referee: " << misc_extra.referee() << std::endl;
    ss << "- State RoboCup: " << misc_extra.robocup() << std::endl;
    ss << "- State Playing: " << misc_extra.playing() << std::endl;
    ss << "- State Search: " << misc_extra.search() << std::endl;
    ss << "- Hardware warnings: " << misc_extra.hardware_warnings() << std::endl;
    ss << std::endl;
  }

  return ss.str();
}
