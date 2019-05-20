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
  bind.bindNew("publishField", publishField, RhIO::Bind::PullOnly)->defaultValue(publishField = true);

  bind.bindNew("supportRatioThreshold", supportRatioThreshold, RhIO::Bind::PullOnly)
      ->defaultValue(supportRatioThreshold = 0.8);

  // Declaration of history entries
  histories.pose("camera");
  histories.pose("trunk");
  histories.pose("self");
  histories.pose("field");
  histories.pose("support");
  histories.pose("supportPitchRoll");

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
  bind.pull();

  if (isReplay)
  {
    // Updating DOFs from replay
    for (const std::string& name : model.getDofNames())
    {
      model.setDof(name, histories.number("read:" + name)->interpolate(replayTimestamp));
    }

    // Updating robot position
    model.supportToWorld = histories.pose("support")->interpolate(replayTimestamp);
    model.supportToWorldPitchRoll = histories.pose("supportPitchRoll")->interpolate(replayTimestamp);

    double left = histories.number("left_pressure_weight")->interpolate(replayTimestamp);
    double right = histories.number("right_pressure_weight")->interpolate(replayTimestamp);
    double total = left + right;
    if (total > 0)
    {
      if (left / total > supportRatioThreshold)
      {
        model.setSupportFoot(model.Left, false);
      }
      if (right / total > supportRatioThreshold)
      {
        model.setSupportFoot(model.Right, false);
      }
    }

    // Updating field position, if localisation replay is enabled
    LocalisationService* localisation = getServices()->localisation;
    if (localisation->isReplay)
    {
      Eigen::Affine3d fieldToWorld = histories.pose("field")->interpolate(replayTimestamp);
      localisation->setPosSelf(fieldToWorld.translation(), rhoban::frameYaw(fieldToWorld.rotation()), 1, 1, false,
                               true);
    }
  }
  else
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
        if (getPressureLeftRatio() > supportRatioThreshold)
        {
          model.setSupportFoot(model.Left, true);
        }
        if (getPressureRightRatio() > supportRatioThreshold)
        {
          model.setSupportFoot(model.Right, true);
        }
      }
    }

    tickLog();
  }

  // Updating low level state
  // XXX: Why this is done in model service?
  tickCheckLowLevelState();

  // Publishing model to ZMQ
  timeSinceLastPublish += elapsed;
  if (timeSinceLastPublish > 0.02 && publish)
  {
    timeSinceLastPublish = 0;
    LocalisationService* localisationService = getServices()->localisation;
    auto ballWorld = localisationService->getBallPosWorld();
    server.setBallPosition(Eigen::Vector3d(ballWorld.x, ballWorld.y, 0));
    if (publishField)
    {
      server.setFieldPose(localisationService->world_from_field);
    }
    else
    {
      server.setFieldPose(Eigen::Affine3d::Identity());
    }
    server.publishModel(model, false);
  }

  bind.push();
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
  histories.pose("field")->pushValue(timestamp, getServices()->localisation->field_from_world);
  histories.pose("support")->pushValue(timestamp, model.supportToWorld);
  histories.pose("supportPitchRoll")->pushValue(timestamp, model.supportToWorldPitchRoll);

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