#include <mutex>
#include <rhoban_utils/angle.h>
#include <rhoban_utils/logging/logger.h>
#include <rhoban_utils/timing/time_stamp.h>
#include <rhoban_utils/timing/benchmark.h>
#include <random>
#include "LocalisationService.h"
#include "ModelService.h"
#include <moves/Walk.h>
#include <moves/Moves.h>
#include <robocup_referee/constants.h>

#include "RhIO.hpp"

#ifdef VISION_COMPONENT
#include "Field/Field.hpp"
#include <Binding/Robocup.hpp>
#include <Binding/LocalisationBinding.hpp>
#endif

static bool block = false;

using ::rhoban_utils::TimeStamp;
using ::rhoban_utils::Benchmark;
using namespace rhoban_geometry;
using namespace rhoban_utils;
using namespace robocup_referee;

#ifdef VISION_COMPONENT
using namespace Vision::Field;
using Vision::Localisation::FieldPF;
#endif

static rhoban_utils::Logger out("localisation_service");
        
LocalisationService::LocalisationService()
  : bind("localisation")
#ifdef VISION_COMPONENT
    ,robocup(NULL), locBinding(NULL)
#endif
{
    lastKick = TimeStamp::now();
    // Ball
    ballPosX = ballPosY = 0;
    ballPosWorld = Eigen::Vector3d(0, 0, 0);
    ballQ = 0;
    ballSpeed = Point(0,0);
    ballTS = TimeStamp::now();
    // Goal
    goalPosX = ballPosY = 0;
    goalCap = 0;
    fieldQ = 0;
    fieldConsistency = 0;
    goalLeftCap = 0;
    goalRightCap = 0;
    goalLeftPosWorld = Eigen::Vector3d(4.5, 0.9, 0.0);
    goalRightPosWorld = Eigen::Vector3d(4.5, -0.9, 0.0);
    // Goal scanner
    goalTargetPan = 0;
    goalTargetTilt = 0;
    // Field
    fieldPosX = 0;
    fieldPosY = 0;
    fieldOrientation = 0;
    fieldOrientationWorld = 0;
    fieldCenterWorld = Eigen::Vector3d(0, 0, 0);
    // Visual compass
    visualCompassActivated = false;

    bind.bindFunc("fakeBall", "Set a fake ball observation in model world",
        &LocalisationService::cmdFakeBall, *this);
    bind.node().newCommand("fakeOpponents", "Fake opponents",
            [this](const std::vector<std::string> &args) {
                return this->cmdFakeOpponents(args);
            });
    bind.bindFunc("fakeLoc", "Set a fake localisation observation in modle world",
        &LocalisationService::cmdFakeLoc, *this);
    bind.bindFunc("moveOnField", "Move the robot on the field",
        &LocalisationService::cmdMoveOnField, *this);
    bind.bindFunc("resetPosition", "Resets the robot position",
        &LocalisationService::cmdResetPosition, *this);

    bind.bindFunc("applyKick", "Apply a kick on the ball filter (for debugging)",
            &LocalisationService::applyKick, *this);

    bind.bindNew("ballQ", ballQ, RhIO::Bind::PushOnly)
        ->comment("Ball quality");
    bind.bindNew("ballX", ballPosX, RhIO::Bind::PushOnly)
        ->comment("Ball X in self [m]");
    bind.bindNew("ballY", ballPosY, RhIO::Bind::PushOnly)
        ->comment("Ball Y in self [m]");
    bind.bindNew("ballFieldX", ballFieldX, RhIO::Bind::PushOnly)
        ->comment("Ball field X [m]");
    bind.bindNew("ballFieldY", ballFieldY, RhIO::Bind::PushOnly)
        ->comment("Ball field Y [m]");
    
    bind.bindNew("opponents", opponents, RhIO::Bind::PushOnly)
        ->comment("Opponent field position as string [m]");
    bind.bindNew("opponentsRadius", opponentsRadius, RhIO::Bind::PullOnly)
        ->comment("Opponent radius [m]")->defaultValue(0.65);

    bind.bindNew("mates", mates, RhIO::Bind::PushOnly)
        ->comment("Team Mates positions as string [m] and [deg]");
    bind.bindNew("teamMatesRadius", teamMatesRadius, RhIO::Bind::PullOnly)
        ->comment("TeamMates radius [m]")->defaultValue(1.5);
    bind.bindNew("sharedOpponents", sharedOpponents, RhIO::Bind::PushOnly)
        ->comment("Opponent positions in field according to mates (as string) [m]");
    
    bind.bindNew("worldBallX", ballPosWorld.x(), RhIO::Bind::PushOnly);
    bind.bindNew("worldBallY", ballPosWorld.y(), RhIO::Bind::PushOnly);
    bind.bindNew("worldLeftGoalX", goalLeftPosWorld.x(), RhIO::Bind::PushOnly);
    bind.bindNew("worldLeftGoalY", goalLeftPosWorld.y(), RhIO::Bind::PushOnly);
    bind.bindNew("worldRightGoalX", goalRightPosWorld.x(), RhIO::Bind::PushOnly);
    bind.bindNew("worldRightGoalY", goalRightPosWorld.y(), RhIO::Bind::PushOnly);
    bind.bindNew("worldFieldX", fieldCenterWorld.x(), RhIO::Bind::PushOnly);
    bind.bindNew("worldFieldY", fieldCenterWorld.y(), RhIO::Bind::PushOnly);

    bind.bindNew("simulateWalk", simulateWalk, RhIO::Bind::PullOnly)
        ->defaultValue(false)->comment("Simulate walking");
    
    bind.bindNew("fieldQ", fieldQ, RhIO::Bind::PushOnly)
        ->comment("Field quality");
    bind.bindNew("fieldConsistency", fieldConsistency, RhIO::Bind::PushOnly)
        ->comment("Field consistency");
    bind.bindNew("goalX", goalPosX, RhIO::Bind::PushOnly)
        ->comment("Goal X");
    bind.bindNew("goalY", goalPosY, RhIO::Bind::PushOnly)
        ->comment("Goal Y");
    bind.bindNew("goalCap", goalCap, RhIO::Bind::PushOnly)
        ->comment("Goal cap");
    bind.bindNew("goalLeftCap", goalLeftCap, RhIO::Bind::PushOnly)
        ->comment("Goal cap");
    bind.bindNew("goalRightCap", goalRightCap, RhIO::Bind::PushOnly)
        ->comment("Goal cap");

    bind.bindNew("fieldX", fieldPosX, RhIO::Bind::PushOnly)
        ->comment("Field X");
    bind.bindNew("fieldY", fieldPosY, RhIO::Bind::PushOnly)
        ->comment("Field Y");
    bind.bindNew("fieldOrientationWorld", fieldOrientationWorld, RhIO::Bind::PushOnly)
        ->comment("Field orientation")->defaultValue(0);
    bind.bindNew("fieldOrientation", fieldOrientation, RhIO::Bind::PushOnly)
        ->comment("Robot orientation in the field referential [deg]")->defaultValue(0);
    
    bind.bindNew("block", block, RhIO::Bind::PullOnly)
        ->comment("Block")->defaultValue(false);
}

Point LocalisationService::getLookBallPosWorld()
{
    mutex.lock();
    auto p = ballLookPosWorld;
    mutex.unlock();
    
    return Point(p.x(), p.y());
}

double LocalisationService::getPenaltyRightGoalCap()
{
    mutex.lock();
    Eigen::Vector3d p;
    p = 
        0.75*getServices()->model->correctedModel().get().frameInSelf("origin", goalLeftPosWorld)
        + 0.25*getServices()->model->correctedModel().get().frameInSelf("origin", goalRightPosWorld);
    mutex.unlock();
   
    return rad2deg(atan2(p.y(), p.x()));
}

double LocalisationService::getPenaltyLeftGoalCap()
{
    mutex.lock();
    Eigen::Vector3d p;
    p = 
        0.25*getServices()->model->correctedModel().get().frameInSelf("origin", goalLeftPosWorld)
        + 0.75*getServices()->model->correctedModel().get().frameInSelf("origin", goalRightPosWorld);
    mutex.unlock();
    
    return rad2deg(atan2(p.y(), p.x()));
}

Point LocalisationService::getBallSpeedSelf()
{
  mutex.lock();
  Leph::HumanoidModel *model = NULL;
  if (Helpers::isFakeMode()) {
    model = &(getServices()->model->goalModel().get());
  } else {
    model = &(getServices()->model->correctedModel().get());
  }
  // Getting 'origin' and 'origin+speedWorld' in self
  Eigen::Vector3d speedVec(ballSpeed.x, ballSpeed.y, 0);
  Eigen::Vector3d start = model->frameInSelf("origin");
  Eigen::Vector3d end = model->frameInSelf("origin", speedVec);
  mutex.unlock();
  Eigen::Vector3d speed = end - start;
  return Point(speed(0), speed(1));
}

Point LocalisationService::getPredictedBallSelf()
{
  return getPredictedBallSelf(TimeStamp::now());
}

Point LocalisationService::getPredictedBallSelf(rhoban_utils::TimeStamp t)
{
  mutex.lock();
  
  double elapsed = diffSec(ballTS, t);  
  Leph::HumanoidModel *model = NULL;
  if (Helpers::isFakeMode()) {
    model = &(getServices()->model->goalModel().get());
  } else {
    model = &(getServices()->model->correctedModel().get());
  }
  // Predicting position in world and then transforming into self
  Eigen::Vector3d speedVec(ballSpeed.x, ballSpeed.y, 0);
  Eigen::Vector3d predictedInWorld = ballPosWorld + speedVec*elapsed;
  Eigen::Vector3d predictedInSelf = model->frameInSelf("origin", predictedInWorld);
  mutex.unlock();
  return Point(predictedInSelf(0), predictedInSelf(1));
  
}

Point LocalisationService::getBallPosSelf()
{
    mutex.lock();
    Eigen::Vector3d left, right;
    Angle left_yaw, right_yaw;

    // Getting left and right feet position and yaw in the world frame
    Leph::HumanoidModel *model = NULL;
    if (Helpers::isFakeMode()) {
      model = &(getServices()->model->goalModel().get());
    } else {
      model = &(getServices()->model->correctedModel().get());
    }

    left = model->position("left_foot_tip", "origin");
    left_yaw = Angle(rad2deg(model->orientationYaw("left_foot_tip", "origin")));
    right = model->position("right_foot_tip", "origin");
    right_yaw = Angle(rad2deg(model->orientationYaw("right_foot_tip", "origin")));

    // Averaging
    Point pos((left.x()+right.x())/2, (left.y()+right.y())/2);
    Angle yaw = Angle::weightedAverage(left_yaw, 1, right_yaw, 1);

    Point p(ballPosWorld.x(), ballPosWorld.y());
    p -= pos;
    p = p.rotation(-yaw);

    mutex.unlock();
    
    return p;
}

Point LocalisationService::getRightGoalPosSelf()
{
    mutex.lock();
    Eigen::Vector3d p;
    if (Helpers::isFakeMode()) {
        p = getServices()->model->goalModel().get().frameInSelf("origin", goalRightPosWorld);
    } else {
        p = getServices()->model->correctedModel().get().frameInSelf("origin", goalRightPosWorld);
    }
    mutex.unlock();
    
    return Point(p.x(), p.y());
}

Point LocalisationService::getGoalPosField()
{
    return Point(Constants::field.fieldLength/2.0, 0);
}
        
Angle LocalisationService::getOurBallToGoalDirSelf()
{
    Point goalField(-Constants::field.fieldLength/2, 0);
    auto ball = getBallPosField();
    auto orientationRad = getFieldOrientation();

    // XXX: This should be more checked
    return 
        Angle(rad2deg(orientationRad)) -
        (goalField-ball).getTheta() 
        ;
}

Point LocalisationService::getFieldPos()
{
    Eigen::Vector3d self;
    if (Helpers::isFakeMode()) {
        self = getServices()->model->goalModel().get().selfInFrame("origin");
    } else {
        self = getServices()->model->correctedModel().get().selfInFrame("origin");
    }
    self -= fieldCenterWorld;
    auto point = Point(self.x(), self.y());

    float x = point.x*cos(fieldOrientationWorld) - point.y*sin(fieldOrientationWorld);
    float y = point.x*sin(fieldOrientationWorld) + point.y*cos(fieldOrientationWorld);

    return Point(x, y);
}

double LocalisationService::getFieldOrientation()
{
    if (Helpers::isFakeMode()) {
        return fieldOrientationWorld + getServices()->model->goalModel().get().orientationYaw("trunk", "origin");
    } else {
        return fieldOrientationWorld + getServices()->model->correctedModel().get().orientationYaw("trunk", "origin");
    }
}

Point LocalisationService::getLeftGoalPosSelf()
{
    mutex.lock();
    Eigen::Vector3d p;
    if (Helpers::isFakeMode()) {
        p = getServices()->model->goalModel().get().frameInSelf("origin", goalLeftPosWorld);
    } else {
        p = getServices()->model->correctedModel().get().frameInSelf("origin", goalLeftPosWorld);
    }
    mutex.unlock();
    
    return Point(p.x(), p.y());
}

Point LocalisationService::getGoalPosSelf()
{
    mutex.lock();
    Eigen::Vector3d p;
    if (Helpers::isFakeMode()) {
        p = 
            0.5*getServices()->model->goalModel().get().frameInSelf("origin", goalLeftPosWorld)
            + 0.5*getServices()->model->goalModel().get().frameInSelf("origin", goalRightPosWorld);
    } else {
        p = 
            0.5*getServices()->model->correctedModel().get().frameInSelf("origin", goalLeftPosWorld)
            + 0.5*getServices()->model->correctedModel().get().frameInSelf("origin", goalRightPosWorld);
    }
    mutex.unlock();
    
    return Point(p.x(), p.y());
}

Point LocalisationService::getGoalPosWorld()
{
    mutex.lock();
    auto p = 0.5*goalLeftPosWorld + 0.5*goalRightPosWorld;
    mutex.unlock();

    return Point(p.x(), p.y());
}


double LocalisationService::getGoalCap()
{
    auto p = getGoalPosSelf();
    return rad2deg(atan2(p.y, p.x));
}

double LocalisationService::getLeftGoalCap()
{
    auto p = getLeftGoalPosSelf();
    return rad2deg(atan2(p.y, p.x));
}

double LocalisationService::getRightGoalCap()
{
    auto p = getRightGoalPosSelf();
    return rad2deg(atan2(p.y, p.x));
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

Point LocalisationService::worldToField(Eigen::Vector3d world)
{
    auto result = getFieldPos();
    auto orientation = getFieldOrientation();

    mutex.lock();
    Eigen::Vector3d p;
    if (Helpers::isFakeMode()) {
        p = getServices()->model->goalModel().get().frameInSelf("origin", world);
    } else {
        p = getServices()->model->correctedModel().get().frameInSelf("origin", world);
    }   
    mutex.unlock();
    auto pos = Point(p.x(), p.y());

    result.x += pos.x*cos(orientation) - pos.y*sin(orientation);
    result.y += pos.x*sin(orientation) + pos.y*cos(orientation);

    return result;
}

std::vector<Point> LocalisationService::getOpponentsField()
{
    std::vector<Point> result;
    mutex.lock();
    auto tmp = opponentsWorld;
    mutex.unlock();

    for (auto &opponent : tmp) {
        if (opponentsAreFake) {
            result.push_back(Point(opponent.x(), opponent.y()));
        } else {
            result.push_back(worldToField(opponent));
        }
    }

    return result;
}

std::map<int, Eigen::Vector3d> LocalisationService::getTeamMatesField()
{
  std::lock_guard<std::mutex> lock(mutex);
  return teamMatesField;
}

void LocalisationService::updateTeamMate(int id, const Eigen::Vector3d & poseInField)
{
  std::lock_guard<std::mutex> lock(mutex);
  teamMatesField[id] = poseInField;
}

void LocalisationService::removeTeamMate(int id)
{
  std::lock_guard<std::mutex> lock(mutex);
  teamMatesField.erase(id);
}

void LocalisationService::updateSharedOpponents(int id,
                                                const std::vector<Eigen::Vector2d> & opponents)
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
    for (auto &opponent : getOpponentsField()) {
        ss << "[" << opponent.x << ", " << opponent.y << "], ";
    }
    ss << "]";

    mutex.lock();
    opponents = ss.str();
    mutex.unlock();
}

void LocalisationService::setOpponentsWorld(const std::vector<Eigen::Vector3d> &pos)
{
    mutex.lock();
    opponentsAreFake = false;
    opponentsWorld = pos;
    mutex.unlock();
    
    updateOpponentsPos();
}

void LocalisationService::updateMatesPos()
{
    std::stringstream ss;
    ss << "{";
    for (const auto & mateEntry : teamMatesField) {
      int robot_id = mateEntry.first;
      Eigen::Vector3d robot_pose = mateEntry.second;
      ss << robot_id << ": "
         << "[" << robot_pose(0) << ", " << robot_pose(1) << ", "
         << rhoban_utils::rad2deg(robot_pose(2)) << "°], ";
    }
    ss << "}";

    mutex.lock();
    mates = ss.str();
    mutex.unlock();
}

std::vector<Eigen::Vector2d> LocalisationService::getSharedOpponents()
{
  std::vector<Eigen::Vector2d> opponents;
  std::lock_guard<std::mutex> lock(mutex);
  for (const auto & opp_entry : sharedOpponentsField) {
    for (const auto & opp : opp_entry.second) {
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
    for (const auto & mateEntry : sharedOpponentsField) {
      int robot_id = mateEntry.first;
      ss << robot_id << ": [";
      for (const Eigen::Vector2d pos : mateEntry.second) {
        ss << "[" << pos(0) << ", " << pos(1) << "], ";
      }
      ss << "],";
    }
    ss << "}";

    mutex.lock();
    sharedOpponents = ss.str();
    mutex.unlock();
}
        
void LocalisationService::setBallWorld(const Eigen::Vector3d &pos,
                                       const Eigen::Vector3d &lookPos,
                                       float quality,
                                       const Point & speed,
                                       const rhoban_utils::TimeStamp & ts)
{
    bind.pull();
    if (block) return;
    mutex.lock();
    ballPosWorld = pos;
    ballLookPosWorld = lookPos;
    ballQ = quality;
    ballSpeed =  speed;
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
        
void LocalisationService::updatePosSelf()
{
    auto p = getGoalPosSelf();
    goalPosX = p.x;
    goalPosY = p.y;
    goalCap = getGoalCap();
    goalLeftCap = getLeftGoalCap();
    goalRightCap = getRightGoalCap();

    fieldOrientation = rad2deg(normalizeRad(getFieldOrientation()));
    auto fp = getFieldPos();
    fieldPosX = fp.x;
    fieldPosY = fp.y;

    bind.push();
}
        
void LocalisationService::setPosSelf(const Eigen::Vector3d &left,
        const Eigen::Vector3d &right, const Eigen::Vector3d &center, float orientation, 
        float quality, float consistency)
{
    bind.pull();

    mutex.lock();

    fieldQ = quality;
    fieldConsistency = consistency;

    if (Helpers::isFakeMode()) {
        fieldCenterWorld = getServices()->model->goalModel().get().selfInFrame("origin", center);
        goalLeftPosWorld = getServices()->model->goalModel().get().selfInFrame("origin", left);
        goalRightPosWorld = getServices()->model->goalModel().get().selfInFrame("origin", right);
    } else {
        fieldCenterWorld = getServices()->model->correctedModel().get().selfInFrame("origin", center);
        goalLeftPosWorld = getServices()->model->correctedModel().get().selfInFrame("origin", left);
        goalRightPosWorld = getServices()->model->correctedModel().get().selfInFrame("origin", right);
    }
    mutex.unlock();
    
    if (Helpers::isFakeMode()) {
        fieldOrientationWorld = orientation - getServices()->model->goalModel().get().orientationYaw("trunk", "origin");
    } else {
        fieldOrientationWorld = orientation - getServices()->model->correctedModel().get().orientationYaw("trunk", "origin");
    }

    updatePosSelf();
}

void LocalisationService::applyKick(float x_, float y_)
{
#ifdef VISION_COMPONENT
  /// Currently has no effect on ballStackFilter
  robocup->applyKick(x_, y_);
#endif

    if (Helpers::isFakeMode()) {
        auto &goalModel = getServices()->model->goalModel().get();
        double yaw = goalModel.getDOF("base_yaw");
        std::random_device rd;
        std::default_random_engine engine(rd());
        std::uniform_real_distribution<double> unif(-0.1, 0.1);
        
        x_ *= 0.01;
        y_ *= 0.01;

        double x = cos(yaw)*x_ - sin(yaw)*y_ + unif(engine);
        double y = sin(yaw)*x_ + cos(yaw)*y_ + unif(engine);

        cmdFakeBall(ballFieldX+x, ballFieldY+y);
    }
}

#ifdef VISION_COMPONENT
void LocalisationService::setRobocup(Vision::Robocup *robocup_)
{
    robocup = robocup_;
}
void LocalisationService::setLocBinding(Vision::LocalisationBinding *locBinding_)
{
    locBinding = locBinding_;
}
#endif
        
void LocalisationService::resetBallFilter()
{
#ifdef VISION_COMPONENT
    // TODO: Do something?
#endif
}
       
void LocalisationService::penaltyReset(float x)
{
#ifdef VISION_COMPONENT
    customFieldReset(x, 0, 0.05, 0, 3);
#endif
}

void LocalisationService::penaltyGoalReset()
{
#ifdef VISION_COMPONENT
    customFieldReset(Constants::field.fieldLength/2, 0, 0.05, 180, 3);
    robocup->robotsClear();
#endif
}

void LocalisationService::goalReset()
{
#ifdef VISION_COMPONENT
    RhIO::Root.setFloat("/Localisation/Field/RobotController/angleExploration", 0.5);
    RhIO::Root.setFloat("/Localisation/Field/RobotController/posExploration", 0.5);
    customFieldReset(-Constants::field.fieldLength/2, 0, 0.01, 0, 1);
    robocup->robotsClear();
#endif
}

void LocalisationService::customFieldReset(double x, double y, double noise, double theta, double thetaNoise)
{
#ifdef VISION_COMPONENT
  locBinding->fieldReset(FieldPF::ResetType::Custom, x, y, noise, theta, thetaNoise);
#endif
}

void LocalisationService::gameStartReset()
{
#ifdef VISION_COMPONENT
  // TODO: Do something for the ball?

  locBinding->fieldReset(FieldPF::ResetType::Borders);
  robocup->robotsClear();
#endif
}

void LocalisationService::kickOffReset()
{
#ifdef VISION_COMPONENT
  robocup->ballReset(Constants::field.centerRadius, 0);
  customFieldReset(-Constants::field.centerRadius, 0, 0.3, 0, 5);
  robocup->robotsClear();
#endif  
}

void LocalisationService::dropBallReset()
{
  //TODO
}

void LocalisationService::fallReset()
{
#ifdef VISION_COMPONENT
    locBinding->fieldReset(FieldPF::ResetType::Fall);
    robocup->ballClear();
    robocup->robotsClear();
#endif
}

void LocalisationService::bordersReset()
{
#ifdef VISION_COMPONENT
  locBinding->fieldReset(FieldPF::ResetType::Borders);
  robocup->robotsClear();
#endif
}

void LocalisationService::setVisualCompassStatus(bool inUse)
{
  visualCompassActivated = inUse;
}

bool LocalisationService::getVisualCompassStatus() const
{
  return visualCompassActivated;
}

bool LocalisationService::isVisionActive() const
{
#ifdef VISION_COMPONENT
  return robocup->activeSource;
#else
  return true;
#endif
}
        
void LocalisationService::enableFieldFilter(bool enable)
{
#ifdef VISION_COMPONENT
    locBinding->enableFieldFilter = enable;
#endif
}

void LocalisationService::isGoalKeeper(bool status)
{
#ifdef VISION_COMPONENT
    locBinding->isGoalKeeper = status;
#endif
}
  

void LocalisationService::resetFieldFilter()
{
#ifdef VISION_COMPONENT
  std::cout << "Reseting filters with locBinding" << locBinding << std::endl;
    locBinding->fieldReset(FieldPF::ResetType::Uniform);
#endif
}

void LocalisationService::resetRobotFilter()
{
#ifdef VISION_COMPONENT
    robocup->robotsClear();
#endif
}
     
std::string LocalisationService::cmdFakeBall(double x, double y)
{
    Eigen::Vector3d posInWorld(x, y, 0.0);
    setBallWorld(posInWorld, posInWorld, 1.0, Point(0,0), TimeStamp::now());
    return "Fake Ball set in world: X=" + std::to_string(x) + std::string(" Y=") + std::to_string(y);
}

std::string LocalisationService::cmdFakeOpponents(std::vector<std::string> args)
{
    mutex.lock();
    opponentsWorld.clear();

    opponentsAreFake = true;
    for (size_t k=0; k<args.size()/2; k++) {
        double x = atof(args[k*2].c_str());
        double y = atof(args[k*2 + 1].c_str());
        Eigen::Vector3d posInWorld(x, y, 0.0);
        opponentsWorld.push_back(posInWorld);
    }
    mutex.unlock();
    updateOpponentsPos();
    
    return "Faked opponents";
}

std::string LocalisationService::cmdFakeLoc(
    double leftX, double leftY, 
    double rightX, double rightY, 
    double fieldX, double fieldY)
{
    Eigen::Vector3d leftGoalInWorld(leftX, leftY, 0.0);
    Eigen::Vector3d rightGoalInWorld(rightX, rightY, 0.0);
    Eigen::Vector3d fieldInWorld(fieldX, fieldY, 0.0);
    setPosSelf(leftGoalInWorld, rightGoalInWorld, fieldInWorld, 0.0, 1.0, 1.0);
    return "Set fake localization in world";
}

std::string LocalisationService::cmdMoveOnField(
        double x, double y, double yaw
        )
{
    fieldQ = 1;

    if (Helpers::isFakeMode()) {
        auto &goalModel = getServices()->model->goalModel().get();
        goalModel.setDOF("base_yaw", 0);
        goalModel.setDOF("base_x", x);
        goalModel.setDOF("base_y", y);
        goalModel.setDOF("base_yaw", yaw);
    } else {
        auto &odometry = getServices()->model->getOdometryModel();
        Eigen::Vector3d pos(x, y, yaw);

        odometry.reset(pos);
    }

    updateBallPos();
    updatePosSelf();

    return "Set fake localization in world";
}

std::string LocalisationService::cmdResetPosition()
{
    if (Helpers::isFakeMode()) {
        auto &goalModel = getServices()->model->goalModel().get();
        goalModel.setDOF("base_x", 0);
        goalModel.setDOF("base_y", 0);
        goalModel.setDOF("base_yaw", 0);
    } else {
        auto &odometry = getServices()->model->getOdometryModel();
        odometry.reset(Eigen::Vector3d(0, 0, 0));
    }

    return "Position reseted";
}

int LocalisationService::getFrames()
{
#ifdef VISION_COMPONENT
    return robocup->getFrames();
#else
    return 0;
#endif
}

std::string LocalisationService::getCameraStatus()
{
#ifdef VISION_COMPONENT
    return robocup->getCameraStatus();
#else
    return "No vision";
#endif
}

double LocalisationService::getLastVisionUpdate()
{
#ifdef VISION_COMPONENT
    return robocup->getLastUpdate();
#else
    return -1;
#endif
}

#ifdef VISION_COMPONENT
void LocalisationService::stealTags(std::vector<int> & indices,
				    std::vector<Eigen::Vector3d> & positions,
				    std::vector<std::pair<float, float> > & centers,
				    std::vector<std::pair<float, float> > & centersUndistorded,
				    double * timestamp) {

  robocup->stealTags(indices, positions, centers, centersUndistorded, timestamp);
}
#endif

        
bool LocalisationService::tick(double elapsed)
{
    bind.pull();

    if (Helpers::isFakeMode()) {
        updatePosSelf();
        updateBallPos();
        updateOpponentsPos();
        updateMatesPos();
        updateSharedOpponentsPos();
    }

    return true;
}
