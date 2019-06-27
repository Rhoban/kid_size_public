#include <iostream>
#include <stdexcept>
#include <rhoban_utils/angle.h>
#include <rhoban_utils/util.h>
#include <rhoban_utils/timing/time_stamp.h>
#include "services/ModelService.h"
#include "services/LocalisationService.h"
#include "moves/Move.h"
#include "scheduler/MoveScheduler.h"
#include <string>
#include <vector>
#include <algorithm>

#include "rhoban_model_learning/model_factory.h"

using namespace rhoban_utils;
using namespace rhoban_model_learning;

ModelService::ModelService()
  : timeSinceLastPublish(0), bind("model"), isReplay(false), histories(60.0), lowLevelState("")
{
  std::unique_ptr<Model> tmp_calibration_model = ModelFactory().buildFromJsonFile("calibration.json");
  calibration_model.reset(dynamic_cast<CalibrationModel*>(tmp_calibration_model.release()));

  if (!calibration_model)
    throw std::logic_error(DEBUG_INFO + "invalid calibration model: check type.");

  cameraModel = calibration_model->getCameraModel();

  odometryYawOffset = 0;
  bind.bindFunc("odometryReset", "Resets the robot odometry", &ModelService::cmdOdometryReset, *this);

  bind.bindNew("publish", publish, RhIO::Bind::PullOnly)->defaultValue(false);
  bind.bindNew("publishField", publishField, RhIO::Bind::PullOnly)->defaultValue(true);

  bind.bindNew("supportRatioThreshold", supportRatioThreshold, RhIO::Bind::PullOnly)->defaultValue(0.8);

  bind.bindNew("useCalibration", useCalibration, RhIO::Bind::PullOnly)->defaultValue(true);
  bind.bindNew("loadCalibration", loadCalibration, RhIO::Bind::PushAndPull)->defaultValue(false);
  bind.bindNew("applyCorrectionInNonCorrectedReplay", applyCorrectionInNonCorrectedReplay, RhIO::Bind::PushAndPull)
      ->defaultValue(false)
      ->comment("If the log was taken with useCalibration=false, then applyCorrectionInNonCorrectedReplay=true applies "
                "the "
                "calibration. Else, applyCorrectionInNonCorrectedReplay should be false.");

  // Declaration of history entries
  histories.pose("camera");
  histories.pose("head_base");
  histories.pose("trunk");
  histories.pose("self");
  histories.pose("field");
  histories.pose("support");
  histories.pose("supportPitchRoll");
  histories.boolean("supportIsLeft");

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

bool ModelService::isFakeMode()
{
  return Helpers::isFakeMode();
}

std::string ModelService::cmdOdometryReset()
{
  odometryYawOffset = getGyroYaw();
  model.resetWorldFrame();
  return "OK";
}

bool ModelService::tick(double elapsed)
{
  bind.pull();

  if (loadCalibration)
  {
    loadCalibration = false;
    std::unique_ptr<Model> model = ModelFactory().buildFromJsonFile("calibration.json");
    calibration_model.reset(dynamic_cast<CalibrationModel*>(model.release()));
    cameraModel = calibration_model->getCameraModel();
  }

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
    model.setSupportFoot(histories.boolean("supportIsLeft")->interpolate(replayTimestamp) ? model.Left : model.Right);

    // Updating field position, if localisation replay is enabled
    LocalisationService* localisation = getServices()->localisation;
    if (localisation->isReplay)
    {
      Eigen::Affine3d fieldToSelf =
          model.selfToWorld().inverse() * histories.pose("field")->interpolate(replayTimestamp).inverse();

      localisation->setPosSelf(fieldToSelf.translation(), -rhoban::frameYaw(fieldToSelf.rotation()), 1, 1, false, true);
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
      if (getPressureLeftRatio() > supportRatioThreshold)
      {
        model.setSupportFoot(model.Left, true);
      }
      if (getPressureRightRatio() > supportRatioThreshold)
      {
        model.setSupportFoot(model.Right, true);
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

Eigen::Affine3d ModelService::cameraToWorld(double timestamp)
{
  Eigen::Affine3d camera_to_world = (histories.pose("camera")->interpolate(timestamp));
  if (not useCalibration)
  {
    return camera_to_world;
  }

  Eigen::Affine3d world_from_head_base = headBaseToWorld(timestamp);
  Eigen::Affine3d world_from_self = selfToWorld(timestamp);
  return applyCalibration(camera_to_world, world_from_head_base, world_from_self);
}

Eigen::Affine3d ModelService::applyCalibration(Eigen::Affine3d camera_to_world, Eigen::Affine3d world_from_head_base,
                                               Eigen::Affine3d world_from_self)
{
  // camera_from_head_base
  Eigen::Affine3d camera_from_world = camera_to_world.inverse();
  Eigen::Affine3d camera_from_head_base = camera_from_world * world_from_head_base;

  // camera_from_self
  Eigen::Affine3d camera_from_self = camera_from_world * world_from_self;

  // apply correction
  Eigen::Affine3d self_from_camera_after_correction =
      calibration_model->getCameraFromSelfAfterCorrection(camera_from_self, camera_from_head_base).inverse();
  Eigen::Affine3d camera_to_world_after_correction = world_from_self * self_from_camera_after_correction;
  return camera_to_world_after_correction;
}

Eigen::Affine3d ModelService::selfToWorld(double timestamp)
{
  return histories.pose("self")->interpolate(timestamp);
}

Eigen::Affine3d ModelService::headBaseToWorld(double timestamp)
{
  return histories.pose("head_base")->interpolate(timestamp);
}

void ModelService::startLogging(const std::string& filename)
{
  histories.startNamedLog(filename);
}

void ModelService::stopLogging(const std::string& filename)
{
  histories.stopNamedLog(filename);
}

void ModelService::loadReplay(const std::string& filename)
{
  isReplay = true;
  histories.loadReplays(filename);
}

void ModelService::setReplayTimestamp(double timestamp)
{
  replayTimestamp = timestamp;
}

Eigen::Vector3d ModelService::odometryDiff(double timestampStart, double timestampEnd)
{
  Eigen::Affine3d startToWorld = selfToWorld(timestampStart);
  Eigen::Affine3d endToWorld = selfToWorld(timestampEnd);
  Eigen::Affine3d endToStart = startToWorld.inverse() * endToWorld;

  return Eigen::Vector3d(endToStart.translation().x(), endToStart.translation().y(),
                         rhoban::frameYaw(endToStart.rotation()));
}

void ModelService::tickLog()
{
  // Logging robot poses
  double timestamp = getLastReadTimestamp();

  // Logging main poses
  histories.pose("camera")->pushValue(timestamp, model.frameToWorld("camera", false));
  histories.pose("head_base")->pushValue(timestamp, model.frameToWorld("head_base", false));
  histories.pose("trunk")->pushValue(timestamp, model.frameToWorld("trunk", false));
  histories.pose("self")->pushValue(timestamp, model.selfToWorld());
  histories.pose("field")->pushValue(timestamp, getServices()->localisation->field_from_world);
  histories.pose("support")->pushValue(timestamp, model.supportToWorld);
  histories.pose("supportPitchRoll")->pushValue(timestamp, model.supportToWorldPitchRoll);
  histories.boolean("supportIsLeft")->pushValue(timestamp, model.supportFoot == model.Left);

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

std::string ModelService::getCameraState()
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

void ModelService::tickCheckLowLevelState()
{
  // Retrieve the RhAL Manager
  RhAL::StandardManager* manager = Helpers::getScheduler()->getManager();
  lowLevelState = "";

  // Checking for camera state
  std::string cameraState = getCameraState();
  if (cameraState != "")
  {
    lowLevelState += cameraState + "; ";
  }

  // Checking for missing devices
  const auto& devs = manager->devContainer();
  for (const auto& it : devs)
  {
    if (!it.second->isPresent())
    {
      lowLevelState += it.first + "; ";
    }
  }
}
