#include <iostream>
#include <stdexcept>
#include <rhoban_utils/angle.h>
#include <rhoban_utils/util.h>
#include <Utils/Angle.h>
#include <Model/NamesModel.h>
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
  , bind("robot_model")
  , isReplay(false)
  , histories(30.0)
  , lowLevelState("")
{
  rhoban_model_learning::CalibrationModel calibration_model;
  calibration_model.loadFile("calibration.json");
  cameraModel = calibration_model.getCameraModel();

  odometryYawOffset = 0;
  bind.bindFunc("odometryReset", "Resets the robot odometry", &RobotModelService::cmdOdometryReset, *this);

  histories.pose("camera");
  histories.pose("trunk");
  histories.pose("self");
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

double RobotModelService::currentTimeStamp()
{
  // Retrieve the RhAL Manager
  RhAL::StandardManager* manager = getScheduler()->getManager();
  // Compute the max timestamp over
  // all DOF and look for communication error
  RhAL::TimePoint timestamp;
  for (const std::string& name : model.getDofNames())
  {
    RhAL::ReadValueFloat value = manager->dev<RhAL::DXL>(name).position().readValue();
    if (timestamp < value.timestamp)
    {
      timestamp = value.timestamp;
    }
  }

  return RhAL::duration_float(timestamp);
}

bool RobotModelService::tick(double elapsed)
{
  RhAL::StandardManager* manager = getScheduler()->getManager();

  for (const std::string& name : model.getDofNames())
  {
    if (Helpers::isFakeMode())
    {
      // Setting written values to the DOFs of the goal model
      double goalValue = RhAL::Deg2Rad(manager->dev<RhAL::DXL>(name).goalPosition().getWrittenValue());
      model.setDof(name, goalValue);
    }
    else
    {
      // Setting read values to the DOFs of the read model
      double readValue = RhAL::Deg2Rad(manager->dev<RhAL::DXL>(name).position().readValue().value);
      model.setDof(name, readValue);
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
      if (getPressureLeftRatio() > 0.9)
      {
        model.setSupportFoot(model.Left, true);
      }
      if (getPressureRightRatio() > 0.9)
      {
        model.setSupportFoot(model.Right, true);
      }
    }
  }

  // Publishing goal model to ZMQ
  timeSinceLastPublish += elapsed;
  if (timeSinceLastPublish > 0.02)
  {
    timeSinceLastPublish = 0;
    LocalisationService* localisationService = getServices()->localisation;
    auto ballWorld = localisationService->getBallPosWorld();
    server.setBallPosition(Eigen::Vector3d(ballWorld.x, ballWorld.y, 0));
    server.publishModel(model, false);
  }

  if (!isReplay)
  {
    // Logging robot poses
    double timestamp = currentTimeStamp();
    histories.pose("camera")->pushValue(timestamp, model.frameToWorld("camera"));
    histories.pose("trunk")->pushValue(timestamp, model.frameToWorld("trunk"));
    histories.pose("self")->pushValue(timestamp, model.selfToWorld());

    // XXX: Log low level informations
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

  std::cout << "~~~~~~" << std::endl;
  std::cout << startToWorld.rotation().matrix() << std::endl;
  std::cout << endToWorld.rotation().matrix() << std::endl;

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