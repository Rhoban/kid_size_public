#include <mutex>
#include <rhoban_team_play/extra_team_play.pb.h>
#include <rhoban_utils/angle.h>
#include <rhoban_utils/logging/logger.h>
#include <rhoban_utils/timing/time_stamp.h>
#include <rhoban_utils/timing/benchmark.h>
#include <random>
#include "RefereeService.h"
#include "TeamPlayService.h"
#include "LocalisationService.h"
#include "ModelService.h"
#include <moves/Walk.h>
#include <moves/Moves.h>
#include <robocup_referee/constants.h>

#include "RhIO.hpp"

#include <Eigen/Geometry>

#include <Binding/Robocup.hpp>
#include <Binding/LocalisationBinding.hpp>
#include <hl_communication/perception.pb.h>

static bool block = false;

using ::rhoban_utils::Benchmark;
using ::rhoban_utils::TimeStamp;
using namespace hl_communication;
using namespace rhoban_geometry;
using namespace rhoban_team_play;
using namespace rhoban_utils;
using namespace robocup_referee;

using Vision::Localisation::FieldPF;

static rhoban_utils::Logger out("localisation_service");

LocalisationService::LocalisationService() : bind("localisation"), robocup(NULL), locBinding(NULL)
{
  lastKick = rhoban_utils::TimeStamp::now();
  isReplay = false;

  // Ball
  ballPosX = ballPosY = 0;
  ballPosWorld = Eigen::Vector3d(0, 0, 0);
  ballQ = 0;
  ballSpeed = Point(0, 0);
  ballTS = rhoban_utils::TimeStamp::now();
  // Field
  fieldPosX = 0;
  fieldPosY = 0;
  fieldOrientation = 0;
  fieldOrientationWorld = 0;
  fieldCenterWorld = Eigen::Vector3d(0, 0, 0);
  fieldQ = 0;
  fieldConsistency = 0;
  consistencyEnabled = false;

  world_from_self = self_from_world = Eigen::Affine3d::Identity();
  world_from_field = field_from_world = Eigen::Affine3d::Identity();

  bind.bindFunc("fakeBall", "Set a fake ball observation in model world", &LocalisationService::cmdFakeBall, *this);
  bind.node().newCommand("fakeOpponents", "Fake opponents",
                         [this](const std::vector<std::string>& args) { return this->cmdFakeOpponents(args); });
  bind.bindFunc("fakeLoc", "Set a fake localisation observation in modle world", &LocalisationService::cmdFakeLoc,
                *this);
  bind.bindFunc("moveOnField", "Move the robot on the field", &LocalisationService::cmdMoveOnField, *this);
  bind.bindFunc("resetPosition", "Resets the robot position", &LocalisationService::cmdResetPosition, *this);

  bind.bindFunc("applyKick", "Apply a kick on the ball filter (for debugging)", &LocalisationService::applyKick, *this);

  bind.bindNew("ballQ", ballQ, RhIO::Bind::PushOnly)->comment("Ball quality");
  bind.bindNew("ballX", ballPosX, RhIO::Bind::PushOnly)->comment("Ball X in self [m]");
  bind.bindNew("ballY", ballPosY, RhIO::Bind::PushOnly)->comment("Ball Y in self [m]");
  bind.bindNew("ballFieldX", ballFieldX, RhIO::Bind::PushOnly)->comment("Ball field X [m]");
  bind.bindNew("ballFieldY", ballFieldY, RhIO::Bind::PushOnly)->comment("Ball field Y [m]");

  bind.bindNew("opponents", opponents, RhIO::Bind::PushOnly)->comment("Opponent field position as string [m]");
  bind.bindNew("opponentsRadius", opponentsRadius, RhIO::Bind::PullOnly)
      ->comment("Opponent radius [m]")
      ->defaultValue(0.3);

  bind.bindNew("teamMatesRadius", teamMatesRadius, RhIO::Bind::PullOnly)
      ->comment("TeamMates radius [m]")
      ->defaultValue(1.1);
  bind.bindNew("sharedOpponents", sharedOpponents, RhIO::Bind::PushOnly)
      ->comment("Opponent positions in field according to mates (as string) [m]");

  bind.bindNew("worldBallX", ballPosWorld.x(), RhIO::Bind::PushOnly);
  bind.bindNew("worldBallY", ballPosWorld.y(), RhIO::Bind::PushOnly);
  bind.bindNew("worldFieldX", fieldCenterWorld.x(), RhIO::Bind::PushOnly);
  bind.bindNew("worldFieldY", fieldCenterWorld.y(), RhIO::Bind::PushOnly);

  bind.bindNew("fieldQ", fieldQ, RhIO::Bind::PushOnly)->comment("Field quality");
  bind.bindNew("fieldConsistency", fieldConsistency, RhIO::Bind::PushOnly)->comment("Field consistency");

  bind.bindNew("fieldX", fieldPosX, RhIO::Bind::PushOnly)->comment("Field X");
  bind.bindNew("fieldY", fieldPosY, RhIO::Bind::PushOnly)->comment("Field Y");
  bind.bindNew("fieldOrientationWorld", fieldOrientationWorld, RhIO::Bind::PushOnly)
      ->comment("Field orientation [rad]")
      ->defaultValue(0);
  bind.bindNew("fieldOrientation", fieldOrientation, RhIO::Bind::PushOnly)
      ->comment("Robot orientation in the field referential [deg]")
      ->defaultValue(0);

  bind.bindNew("replayLocalisation", isReplay, RhIO::Bind::PullOnly)->defaultValue(isReplay);

  bind.bindNew("block", block, RhIO::Bind::PullOnly)->comment("Block")->defaultValue(false);

  bind.bindNew("opponentsAreFake", opponentsAreFake, RhIO::Bind::PullOnly)
      ->comment("Are opponents based on fake data?")
      ->defaultValue(false);
}

Eigen::Affine3d LocalisationService::getSupportFootToSelf()
{
  auto& model = getServices()->model->model;
  Eigen::Affine3d supportFootToSelf = Eigen::Affine3d::Identity();
  supportFootToSelf.translation().y() =
      (model.supportFoot == rhoban::HumanoidModel::Left ? Helpers::footYOffset() : -Helpers::footYOffset());

  return supportFootToSelf * model.supportToWorld.inverse();
}

Point LocalisationService::getBallSpeedSelf()
{
  mutex.lock();
  Eigen::Vector3d speed_world(ballSpeed.x, ballSpeed.y, 0);
  mutex.unlock();
  Eigen::Vector3d speed_self = self_from_world.linear() * speed_world;
  return Point(speed_self.x(), speed_self.y());
}

Point LocalisationService::getPredictedBallSelf()
{
  return getPredictedBallSelf(rhoban_utils::TimeStamp::now());
}

Point LocalisationService::getPredictedBallSelf(rhoban_utils::TimeStamp t, bool supportBasedSelf)
{
  // Extracting consistent information
  mutex.lock();
  double elapsed = diffSec(ballTS, t);
  Eigen::Vector3d ball_world = ballPosWorld;
  Eigen::Vector3d ball_speed_world(ballSpeed.x, ballSpeed.y, 0);
  mutex.unlock();
  // Predicting position in world and then transforming into self
  Eigen::Vector3d predicted_in_world = ball_world + ball_speed_world * elapsed;
  Eigen::Vector3d predicted_in_self;

  if (supportBasedSelf)
  {
    predicted_in_self = getSupportFootToSelf() * predicted_in_world;
  }
  else
  {
    predicted_in_self = self_from_world * predicted_in_world;
  }

  return Point(predicted_in_self.x(), predicted_in_self.y());
}

Point LocalisationService::getBallPosSelf(bool supportBasedSelf)
{
  mutex.lock();
  Eigen::Vector3d ball_pos_self;
  if (supportBasedSelf)
  {
    ball_pos_self = getSupportFootToSelf() * ballPosWorld;
  }
  else
  {
    ball_pos_self = self_from_world * ballPosWorld;
  }
  mutex.unlock();
  return Point(ball_pos_self.x(), ball_pos_self.y());
}

Point LocalisationService::getOurGoalPosField()
{
  return Point(-Constants::field.field_length / 2.0, 0);
}

Point LocalisationService::getGoalPosField()
{
  return Point(Constants::field.field_length / 2.0, 0);
}

Angle LocalisationService::getOurBallToGoalDirSelf()
{
  Point goalField(-Constants::field.field_length / 2, 0);
  auto ball = getBallPosField();
  auto orientationRad = getFieldOrientation();

  // XXX: This should be more checked
  return Angle(rad2deg(orientationRad)) - (goalField - ball).getTheta();
}

Point LocalisationService::getFieldPos()
{
  Eigen::Vector3d robot_pos_in_field = field_from_world * world_from_self * Eigen::Vector3d::Zero();
  return Point(robot_pos_in_field.x(), robot_pos_in_field.y());
}

double LocalisationService::getFieldOrientation()
{
  Eigen::Vector3d field_dir_in_world(cos(fieldOrientationWorld), sin(fieldOrientationWorld), 0);
  Eigen::Vector3d field_dir_in_self = self_from_world.linear() * field_dir_in_world;
  return -atan2(field_dir_in_self.y(), field_dir_in_self.x());
}

Point LocalisationService::getBallPosWorld()
{
  mutex.lock();
  auto result = Point(ballPosWorld.x(), ballPosWorld.y());
  mutex.unlock();

  return result;
}

Point LocalisationService::getBallPosField()
{
  return worldToField(ballPosWorld);
}

Point LocalisationService::worldToField(const Eigen::Vector3d& pos_in_world)
{
  Eigen::Vector3d pos_in_field = field_from_world * pos_in_world;
  return Point(pos_in_field.x(), pos_in_field.y());
}

Eigen::Vector3d LocalisationService::fieldToWorld(const Eigen::Vector3d& pos_in_field)
{
  return world_from_field * pos_in_field;
}

std::vector<Eigen::Vector3d> LocalisationService::getOpponentsSelf()
{
  std::vector<Eigen::Vector3d> result;
  mutex.lock();
  std::vector<Eigen::Vector3d> tmp = opponentsWorld;
  mutex.unlock();

  for (const Eigen::Vector3d& opponent : tmp)
  {
    if (opponentsAreFake)
    {
      result.push_back(self_from_world * world_from_field * opponent);
    }
    else
    {
      result.push_back(self_from_world * opponent);
    }
  }
  return result;
}

std::vector<Point> LocalisationService::getOpponentsField()
{
  std::vector<Point> result;
  mutex.lock();
  auto tmp = opponentsWorld;
  mutex.unlock();

  for (auto& opponent : tmp)
  {
    if (opponentsAreFake)
    {
      result.push_back(Point(opponent.x(), opponent.y()));
    }
    else
    {
      result.push_back(worldToField(opponent));
    }
  }

  return result;
}

std::map<int, Eigen::Vector3d> LocalisationService::getTeamMatesField()
{
  std::map<int, Eigen::Vector3d> mates;
  auto teamPlay = getServices()->teamPlay;
  auto referee = getServices()->referee;
  for (auto& entry : teamPlay->allInfo())
  {
    const RobotMsg& info = entry.second;
    const PerceptionExtra perception_extra = extractPerceptionExtra(info.perception());
    int info_id = info.robot_id().robot_id();
    if (info_id != teamPlay->myId() && !isOutdated(info) && !referee->isPenalized(info_id) &&
        perception_extra.field().valid())
    {
      const PoseDistribution pose = info.perception().self_in_field(0).pose();
      mates[info_id] = Eigen::Vector3d(pose.position().x(), pose.position().y(), pose.dir().mean());
    }
  }
  return mates;
}

void LocalisationService::updateSharedOpponents(int id, const std::vector<Eigen::Vector2d>& opponents)
{
  std::lock_guard<std::mutex> lock(mutex);
  sharedOpponentsField[id] = opponents;
}

void LocalisationService::removeSharedOpponentProvider(int id)
{
  std::lock_guard<std::mutex> lock(mutex);
  sharedOpponentsField.erase(id);
}

void LocalisationService::updateBallPos()
{
  auto p = getBallPosSelf();
  ballPosX = p.x;
  ballPosY = p.y;

  auto pf = getBallPosField();
  ballFieldX = pf.x;
  ballFieldY = pf.y;

  Point speed = getBallSpeedSelf();
  ballSpeedXInSelf = speed.x;
  ballSpeedYInSelf = speed.y;

  Point predictedPos = getPredictedBallSelf();
  ballPredictedX = predictedPos.x;
  ballPredictedY = predictedPos.y;

  bind.push();
}

void LocalisationService::updateOpponentsPos()
{
  std::stringstream ss;
  ss << "[";
  for (auto& opponent : getOpponentsField())
  {
    ss << "[" << opponent.x << ", " << opponent.y << "], ";
  }
  ss << "]";

  mutex.lock();
  opponents = ss.str();
  mutex.unlock();
}

void LocalisationService::setOpponentsWorld(const std::vector<Eigen::Vector3d>& pos)
{
  if (!opponentsAreFake)
  {
    mutex.lock();
    opponentsWorld = pos;
    mutex.unlock();

    updateOpponentsPos();
  }
}
//
// void LocalisationService::updateMatesPos()
// {
//     std::stringstream ss;
//     ss << "{";
//     for (const auto & mateEntry : teamMatesField) {
//       int robot_id = mateEntry.first;
//       Eigen::Vector3d robot_pose = mateEntry.second;
//       ss << robot_id << ": "
//          << "[" << robot_pose(0) << ", " << robot_pose(1) << ", "
//          << rhoban_utils::rad2deg(robot_pose(2)) << "°], ";
//     }
//     ss << "}";
//
//     mutex.lock();
//     mates = ss.str();
//     mutex.unlock();
// }

std::vector<Eigen::Vector2d> LocalisationService::getSharedOpponents()
{
  std::vector<Eigen::Vector2d> opponents;
  std::lock_guard<std::mutex> lock(mutex);
  for (const auto& opp_entry : sharedOpponentsField)
  {
    for (const auto& opp : opp_entry.second)
    {
      opponents.push_back(opp);
    }
  }
  return opponents;
  // TODO: eventually merge obstacles
}

void LocalisationService::updateSharedOpponentsPos()
{
  std::stringstream ss;
  ss << "{";
  for (const auto& mateEntry : sharedOpponentsField)
  {
    int robot_id = mateEntry.first;
    ss << robot_id << ": [";
    for (const Eigen::Vector2d pos : mateEntry.second)
    {
      ss << "[" << pos(0) << ", " << pos(1) << "], ";
    }
    ss << "],";
  }
  ss << "}";

  mutex.lock();
  sharedOpponents = ss.str();
  mutex.unlock();
}

void LocalisationService::setBallWorld(const Eigen::Vector3d& pos, float quality, const Point& speed,
                                       const rhoban_utils::TimeStamp& ts)
{
  bind.pull();
  if (block)
    return;
  mutex.lock();
  ballPosWorld = pos;
  ballQ = quality;
  ballSpeed = speed;
  ballTS = ts;
  mutex.unlock();

  updateBallPos();
}

void LocalisationService::setNoBall()
{
  bind.pull();
  mutex.lock();
  ballQ = 0;
  mutex.unlock();
  bind.push();
}
void LocalisationService::setClusters(const std::vector<hl_communication::WeightedPose>& candidates)
{
  posFromClusters = candidates;
}

std::vector<hl_communication::WeightedPose> LocalisationService::getPositionInClusters()
{
  if (Helpers::isFakeMode())
  {
    fakeWeightedPose.mutable_pose()->mutable_position()->set_x(fieldPosX);
    fakeWeightedPose.mutable_pose()->mutable_position()->set_y(fieldPosY);
    fakeWeightedPose.mutable_pose()->mutable_dir()->set_mean(deg2rad(fieldOrientation));
    fakeWeightedPose.set_probability(1);

    std::vector<hl_communication::WeightedPose> tmp;
    tmp.push_back(fakeWeightedPose);
    return tmp;
  }
  else
  {
    return posFromClusters;
  }
}

void LocalisationService::updatePosSelf()
{
  updateFieldWorldTransforms();
  fieldOrientation = rad2deg(normalizeRad(getFieldOrientation()));
  auto fp = getFieldPos();
  fieldPosX = fp.x;
  fieldPosY = fp.y;

  bind.push();
}

void LocalisationService::setPosSelf(const Eigen::Vector3d& center_in_self, float orientation, float quality,
                                     float consistency, bool consistencyEnabled_, bool replayValue)
{
  if (isReplay != replayValue)
  {
    return;
  }

  bind.pull();

  fieldQ = quality;
  fieldConsistency = consistency;
  consistencyEnabled = consistencyEnabled_;

  fieldCenterWorld = world_from_self * center_in_self;

  Eigen::Vector3d field_dir_in_self(cos(-orientation), sin(-orientation), 0);
  Eigen::Vector3d field_dir_in_world = world_from_self.linear() * field_dir_in_self;

  fieldOrientationWorld = atan2(field_dir_in_world.y(), field_dir_in_world.x());

  updatePosSelf();
}

void LocalisationService::applyKick(float x_, float y_)
{
  if (Helpers::isFakeMode() && !Helpers::isPython)
  {
    double yaw = rhoban::frameYaw(getSupportFootToSelf().rotation());
    std::random_device rd;
    std::default_random_engine engine(rd());
    std::uniform_real_distribution<double> unif(-0.1, 0.1);

    double x = cos(yaw) * x_ - sin(yaw) * y_ + unif(engine);
    double y = sin(yaw) * x_ + cos(yaw) * y_ + unif(engine);

    cmdFakeBall(ballFieldX + x, ballFieldY + y);
  }
}

void LocalisationService::setRobocup(Vision::Robocup* robocup_)
{
  robocup = robocup_;
}
void LocalisationService::setLocBinding(Vision::LocalisationBinding* locBinding_)
{
  locBinding = locBinding_;
}

void LocalisationService::resetBallFilter()
{
  // TODO: Do something?
}

void LocalisationService::penaltyReset(float x)
{
  customFieldReset(x, 0, 0.05, 0, 3);
}

void LocalisationService::penaltyGoalReset()
{
  customFieldReset(Constants::field.field_length / 2, 0, 0.05, 180, 3);

  if (NULL != robocup)
  {
    robocup->robotsClear();
  }
}

void LocalisationService::goalReset()
{
  customFieldReset(-Constants::field.field_length / 2, 0, 0.01, 0, 1);

  if (NULL != robocup)
  {
    robocup->robotsClear();
  }
}

void LocalisationService::customFieldReset(double x, double y, double noise, double theta, double thetaNoise)
{
  if (NULL != locBinding)
  {
    locBinding->fieldReset(FieldPF::ResetType::Custom, x, y, noise, theta, thetaNoise);
  }
}

void LocalisationService::fallReset()
{
  if (NULL != locBinding)
  {
    locBinding->fieldReset(FieldPF::ResetType::Fall);
  }

  if (NULL != robocup)
  {
    robocup->ballClear();
    robocup->robotsClear();
  }
}

void LocalisationService::bordersReset()
{
  if (NULL != locBinding)
  {
    locBinding->fieldReset(FieldPF::ResetType::Borders);
  }
  if (NULL != robocup)
  {
    robocup->robotsClear();
  }
}

bool LocalisationService::isVisionActive() const
{
  if (NULL != robocup)
  {
    double lastFrame = diffSec(robocup->lastTS, rhoban_utils::TimeStamp::now());
    return lastFrame < 3;
  }
  else
  {
    return true;
  }
}

void LocalisationService::setGoalKeeper(bool status)
{
  if (NULL != locBinding)
  {
    locBinding->isGoalKeeper = status;
  }
}

void LocalisationService::resetFieldFilter()
{
  if (NULL != locBinding)
  {
    std::cout << "Reseting filters with locBinding" << locBinding << std::endl;
    locBinding->fieldReset(FieldPF::ResetType::Uniform);
  }
}

void LocalisationService::resetRobotFilter()
{
  if (NULL != robocup)
  {
    robocup->robotsClear();
  }
}

std::string LocalisationService::cmdFakeBall(double x, double y)
{
  Eigen::Vector3d posInWorld(x, y, 0.0);
  setBallWorld(posInWorld, 1.0, Point(0, 0), rhoban_utils::TimeStamp::now());
  return "Fake Ball set in world: X=" + std::to_string(x) + std::string(" Y=") + std::to_string(y);
}

std::string LocalisationService::cmdFakeOpponents(std::vector<std::string> args)
{
  out.log("Faking opponents");
  mutex.lock();
  opponentsWorld.clear();

  opponentsAreFake = true;
  for (size_t k = 0; k < args.size() / 2; k++)
  {
    double x = atof(args[k * 2].c_str());
    double y = atof(args[k * 2 + 1].c_str());
    Eigen::Vector3d posInWorld(x, y, 0.0);
    opponentsWorld.push_back(posInWorld);
  }
  mutex.unlock();
  updateOpponentsPos();

  return "Faked opponents";
}

std::string LocalisationService::cmdFakeLoc(double fieldX, double fieldY, double orientation)
{
  Eigen::Vector3d fieldInWorld(fieldX, fieldY, 0.0);
  setPosSelf(fieldInWorld, orientation, 1.0, 1.0);
  return "Set fake localization in world";
}

// XXX: Actually, this method name is only real in simulation mode
//      because it sets the robot at some world position and not field position
std::string LocalisationService::cmdMoveOnField(double x, double y, double yaw)
{
  fieldQ = 1;

  // Computing the new selfInWorld target
  Eigen::Affine3d selfToWorld = Eigen::Affine3d::Identity();
  selfToWorld.translation().x() = x;
  selfToWorld.translation().y() = y;
  selfToWorld.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();

  // Updating supportToWorld accordingly
  rhoban::HumanoidModel& model = getServices()->model->model;
  Eigen::Affine3d supportToSelf = model.selfToWorld().inverse() * model.supportToWorld;
  model.supportToWorld = selfToWorld * supportToSelf;

  model.updateImu();

  updateBallPos();
  updatePosSelf();

  return "Set fake localization in world";
}

std::string LocalisationService::cmdResetPosition()
{
  return cmdMoveOnField(0, 0, 0);
}

int LocalisationService::getFrames()
{
  if (NULL != robocup)
  {
    return robocup->getFrames();
  }
  else
  {
    return 0;
  }
}

std::string LocalisationService::getCameraStatus()
{
  if (NULL != robocup)
  {
    return robocup->getCameraStatus();
  }
  else
  {
    return "No vision";
  }
}

double LocalisationService::getLastVisionUpdate()
{
  if (NULL != robocup)
  {
    return robocup->getLastUpdate();
  }
  else
  {
    return -1;
  }
}

bool LocalisationService::tick(double elapsed)
{
  bind.pull();

  updateSelfWorldTransforms();

  if (Helpers::isFakeMode())
  {
    if (!Helpers::isPython)
    {
      updatePosSelf();
      updateBallPos();
    }
    updateOpponentsPos();
    updateSharedOpponentsPos();
  }

  bind.push();

  return true;
}

void LocalisationService::updateFieldWorldTransforms()
{
  Eigen::Matrix3d world_from_field_orientation;
  world_from_field_orientation = Eigen::AngleAxisd(fieldOrientationWorld, Eigen::Vector3d::UnitZ());
  world_from_field = Eigen::Translation3d(fieldCenterWorld) * Eigen::Affine3d(world_from_field_orientation);
  field_from_world = world_from_field.inverse();
}

void LocalisationService::updateSelfWorldTransforms()
{
  world_from_self = getServices()->model->model.selfToWorld();
  self_from_world = world_from_self.inverse();
}

Angle LocalisationService::getSelfOrientationInWorld()
{
  Eigen::Vector3d trunk_dir = world_from_self.linear() * Eigen::Vector3d::UnitX();
  return Angle::fromXY(trunk_dir.x(), trunk_dir.y());
}
