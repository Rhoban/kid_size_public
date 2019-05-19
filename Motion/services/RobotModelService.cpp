#include <iostream>
#include <stdexcept>
#include <rhoban_utils/angle.h>
#include <rhoban_utils/util.h>
#include <rhoban_utils/timing/time_stamp.h>
#include "services/RobotModelService.h"
#include "services/LocalisationService.h"
#include "rhoban_model_learning/humanoid_models/calibration_model.h"
#include "moves/Move.h"
#include "scheduler/MoveScheduler.h"
#include <string>
#include <vector>
#include <algorithm>

using namespace rhoban_utils;

/**
 * Head
 * - Camera look at etc.
 *
 * Localisation service
 *
 * Vision
 * - Pose de la camera avec timestamp
 * - Pose du trunk avec timestamp
 * - Camera model XXX
 *
 * Binding Robocup: Logs
 *  - DOF goals
 *  - DOF reads
 *  - Pressure
 *  - IMU
 *  - Camera pose
 */
RobotModelService::RobotModelService()
  : odometryEnabled(false)
  , odometryUpdated(false)
  , timeSinceLastPublish(0)
  , bind("model")
  , isReplay(false)
  , histories(30.0)
  , lowLevelState("")
{
  rhoban_model_learning::CalibrationModel calibration_model;
  calibration_model.loadFile("calibration.json");
  cameraModel = calibration_model.getCameraModel();

  odometryYawOffset = 0;
  bind.bindFunc("odometryReset", "Resets the robot odometry", &RobotModelService::cmdOdometryReset, *this);

  bind.bindNew("publish", publish, RhIO::Bind::PullOnly)->defaultValue(publish = false);

  // Declaration of history entries
  histories.pose("camera");
  histories.pose("trunk");
  histories.pose("self");

  // DOFs reading and writing
  for (auto& name : model.getDofNames())
  {
    histories.number("read:" + name);
    histories.number("goal:" + name);
  }

  // IMU
  histories.angle("imu_gyro_yaw");
  histories.angle("imu_pitch");
  histories.angle("imu_roll");

  // Pressure sensors
  histories.number("left_pressure_weight");
  histories.number("left_pressure_x");
  histories.number("left_pressure_y");
  histories.number("right_pressure_weight");
  histories.number("right_pressure_x");
  histories.number("right_pressure_y");
}

bool RobotModelService::isFakeMode()
{
  return Helpers::isFakeMode();
}

std::string RobotModelService::cmdOdometryReset()
{
  odometryYawOffset = getGyroYaw();
  model.resetWorldFrame();
  return "OK";
}

bool RobotModelService::tick(double elapsed)
{
  for (const std::string& name : model.getDofNames())
  {
    if (Helpers::isFakeMode())
    {
      // Setting written values to the DOFs of the goal model
      model.setDof(name, deg2rad(getGoalAngle(name)));
    }
    else
    {
      // Setting read values to the DOFs of the read model
      model.setDof(name, deg2rad(getAngle(name)));
    }
  }

  if (!Helpers::isFakeMode() || Helpers::isPython)
  {
    // Using the IMU
    model.setImu(true, getGyroYaw() - odometryYawOffset, getPitch(), getRoll());

    // Integrating odometry
    if (odometryEnabled)
    {
      odometryUpdated = true;
      if (getPressureLeftRatio() > 0.6)
      {
        model.setSupportFoot(model.Left, true);
      }
      if (getPressureRightRatio() > 0.6)
      {
        model.setSupportFoot(model.Right, true);
      }
    }
  }

  // Publishing goal model to ZMQ
  timeSinceLastPublish += elapsed;
  if (timeSinceLastPublish > 0.02 && publish)
  {
    timeSinceLastPublish = 0;
    LocalisationService* localisationService = getServices()->localisation;
    auto ballWorld = localisationService->getBallPosWorld();
    server.setBallPosition(Eigen::Vector3d(ballWorld.x, ballWorld.y, 0));
    server.setFieldPose(localisationService->world_from_field);
    server.publishModel(model, false);
  }

  if (!isReplay)
  {
    tickLog();
  }

  // Updating low level state
  // XXX: Why this is done in model service?
  tickCheckLowLevelState();

  return true;
}

Eigen::Affine3d RobotModelService::cameraToWorld(double timestamp)
{
  return histories.pose("camera")->interpolate(timestamp);
}

Eigen::Affine3d RobotModelService::selfToWorld(double timestamp)
{
  return histories.pose("self")->interpolate(timestamp);
}

void RobotModelService::startLogging(const std::string& filename)
{
  histories.startNamedLog(filename);
}

void RobotModelService::stopLogging(const std::string& filename)
{
  histories.stopNamedLog(filename);
}

void RobotModelService::loadReplay(const std::string& filename)
{
  isReplay = true;
  histories.loadReplays(filename);
}

void RobotModelService::setReplayTimestamp(double timestamp)
{
  replayTimestamp = timestamp;
}

Eigen::Vector3d RobotModelService::odometryDiff(double timestampStart, double timestampEnd)
{
  Eigen::Affine3d startToWorld = selfToWorld(timestampStart);
  Eigen::Affine3d endToWorld = selfToWorld(timestampEnd);
  Eigen::Affine3d endToStart = startToWorld.inverse() * endToWorld;

  return Eigen::Vector3d(endToStart.translation().x(), endToStart.translation().y(),
                         rhoban::frameYaw(endToStart.rotation()));
}

void RobotModelService::enableOdometry(bool enabled)
{
  odometryEnabled = enabled;
}

bool RobotModelService::wasOdometryUpdated()
{
  bool result = odometryUpdated;
  odometryUpdated = false;
  return result;
}

void RobotModelService::tickLog()
{
  // Logging robot poses
  double timestamp = getLastReadTimestamp();

  // Logging main poses
  histories.pose("camera")->pushValue(timestamp, model.frameToWorld("camera", false));
  histories.pose("trunk")->pushValue(timestamp, model.frameToWorld("trunk", false));
  histories.pose("self")->pushValue(timestamp, model.selfToWorld());

  // Logging DOFs
  for (auto& name : model.getDofNames())
  {
    histories.number("read:" + name)->pushValue(timestamp, deg2rad(getAngle(name)));
    histories.number("goal:" + name)->pushValue(timestamp, deg2rad(getGoalAngle(name)));
  }

  // Logging IMU
  histories.number("imu_gyro_yaw")->pushValue(timestamp, getGyroYaw());
  histories.number("imu_pitch")->pushValue(timestamp, getPitch());
  histories.number("imu_roll")->pushValue(timestamp, getRoll());

  // Loggin pressure sensors
  double weight = getPressureWeight();
  histories.number("left_pressure_weight")->pushValue(timestamp, weight * getPressureLeftRatio());
  histories.number("left_pressure_x")->pushValue(timestamp, weight * getLeftPressureX());
  histories.number("left_pressure_y")->pushValue(timestamp, weight * getLeftPressureY());

  histories.number("right_pressure_weight")->pushValue(timestamp, weight * getPressureRightRatio());
  histories.number("right_pressure_x")->pushValue(timestamp, weight * getRightPressureX());
  histories.number("right_pressure_y")->pushValue(timestamp, weight * getRightPressureY());
}

std::string RobotModelService::getCameraState()
{
  auto loc = getServices()->localisation;
  // Check camera state
  std::string cameraState = "No vision";

  double lastUpdate = loc->getLastVisionUpdate();
  if (lastUpdate > 0)
  {
    std::string statusVision = loc->getCameraStatus();
    double lastUpdateVision = lastUpdate / 1000.0;
    if (statusVision.find("lost") != std::string::npos)
    {
      cameraState = "Vision lost " + std::to_string(lastUpdateVision);
    }
    else
    {
      cameraState = "";
    }
  }

  return cameraState;
}

void RobotModelService::tickCheckLowLevelState()
{
  // Retrieve the RhAL Manager
  RhAL::StandardManager* manager = Helpers::getScheduler()->getManager();
  lowLevelState = "";

  // Checking for camera state
  std::string cameraState = getCameraState();
  if (cameraState != "")
  {
    lowLevelState += cameraState;
    lowLevelState += '\n';
  }

  // Checking for missing devices
  const auto& devs = manager->devContainer();
  for (const auto& it : devs)
  {
    if (!it.second->isPresent())
    {
      lowLevelState += it.first + "\n";
    }
  }
}