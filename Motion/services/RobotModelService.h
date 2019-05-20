#pragma once

#include "Service.h"
#include "rhoban_utils/history/history.h"
#include "robot_model/humanoid_model.h"
#include "robot_model/humanoid_server.h"
#include "robot_model/camera_model.h"

class Move;

class RobotModelService : public Service
{
public:
  /**
   * Initialization
   */
  RobotModelService();

  bool tick(double elapsed);

  double currentTimeStamp();

  // Are we currently in fake mode ?
  bool isFakeMode();

  // Should we update the base ?
  Eigen::Vector3d odometryDiff(double timestampStart, double timestampEnd);
  void enableOdometry(bool enabled);
  bool wasOdometryUpdated();
  bool odometryEnabled;
  bool odometryUpdated;

  // Models
  rhoban::HumanoidModel model;

  // Publishing
  rhoban::HumanoidServer server;
  double timeSinceLastPublish;
  bool publish;
  bool publishField;

  // RhIO commands
  RhIO::Bind bind;
  double odometryYawOffset;
  std::string cmdOdometryReset();

  // Returns camera in world transformation
  Eigen::Affine3d cameraToWorld(double timestamp);
  Eigen::Affine3d selfToWorld(double timestamp);
  rhoban::CameraModel cameraModel;

  // Logging
  double replayTimestamp;
  bool isReplay;
  void startLogging(const std::string& filename);
  void stopLogging(const std::string& filename);
  void loadReplay(const std::string& filename);
  void setReplayTimestamp(double timestamp);
  rhoban_utils::HistoryCollection histories;
  void tickLog();

  // XXX: This has nothing to do here...
  std::string lowLevelState;
  std::string getCameraState();
  void tickCheckLowLevelState();
};
