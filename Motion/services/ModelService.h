#pragma once

#include <rhoban_model_learning/humanoid_models/vision_correction_model.h>

#include <Eigen/Dense>
#include <string>
#include <map>
#include <RhAL.hpp>
#include "services/Service.h"
#include <rhoban_utils/history/history.h>
#include <Model/HumanoidFixedPressureModel.hpp>
#include <Model/HumanoidFixedModel.hpp>
#include <Odometry/Odometry.hpp>
#include <RhIO.hpp>
#include <Utils/Euler.h>
#include <Utils/FileEigen.h>
#include <Utils/FileMap.h>
#include <iostream>
class Move;


///Initialize and return an Humanoid Model where the robot type is retrieved
///from RhIO and the geometry parameter from the camera model file.
///ModelType should be either HumanoidFixedModel or HumanoidFixedPressureModel.
template <typename ModelType>
ModelType InitHumanoidModel()
{
  //Retrieve the robot model type from RhIO configuration
  Leph::RobotType robotType;
  if (RhIO::Root.getValueType("/model/modelType")
      == RhIO::TypeStr
    ) {
    std::string type =
      RhIO::Root.getStr("/model/modelType");
    if (type == "sigmaban") {
      robotType = Leph::SigmabanModel;
    } else if (type  == "grosban") {
      robotType = Leph::GrosbanModel;
    } else {
      throw std::logic_error(
        "ModelService invalid modelType in RhIO: "
        + type);
    }
  } else {
    std::cout << "WARNING ModelService no model type defined. "
              << "Default is SigmabanModel" << std::endl;
    robotType = Leph::SigmabanModel;
  }

  // Reading the correection to bring to the model
  rhoban_model_learning::VisionCorrectionModel correction_model;
  correction_model.loadFile("VCM.json");

  // Importing geometry data from a default model
  // Current intialization is based on the fact that initial root doesn't matter
  Leph::HumanoidModel tmpModel(robotType, "left_foot_tip");
  Eigen::MatrixXd geometryData;
  std::map<std::string, size_t> geometryName;
  geometryData = tmpModel.getGeometryData();
  geometryName = tmpModel.getGeometryName();

  // Modification of geometry data
  geometryData.block(geometryName.at("camera"),0,1,3) +=
    correction_model.getCameraOffsetsRad().transpose();
  geometryData.block(geometryName.at("head_yaw"),0,1,3) +=
    correction_model.getNeckOffsetsRad().transpose();

  //Initialize and return the model
  return ModelType (robotType, Eigen::MatrixXd(), {},
                    geometryData, geometryName);
}

/**
 * ModelService
 *
 * Provide interface to Robot Model.
 * There is one model for goal position and
 * inverse kinematic computation and one
 * model for current read state value.
 */
class ModelService : public Service
{
public:

  /**
   * Initialization
   */
  ModelService();

  /**
   * Implement ElapseTick.
   * Update Read model using
   * RhAL low level information
   */
  bool tick(double elapsed) override;

  /**
   * Send target motor position to RhAL low level.
   * Applay all DOF of only apply both arms,
   * both legs, head or left/right leg.
   */
  void flushAll(double gain=1.0);
  void flushArms(double gain=1.0);
  void flushHead(double gain=1.0);
  void flushLegs(double gain=1.0);
  void flushLeftLeg(double gain=1.0);
  void flushRightLeg(double gain=1.0);

  /**
   * Flush the goal model for
   * given DOFs
   */
  void flush(
    bool doHead,
    bool doLeftArm,
    bool doRightArm,
    bool doLeftLeg,
    bool doRightLeg,
    double gain=1.0);

  /**
   * Access to Goal and Read  and odometry
   * corrected model
   */
  Leph::HumanoidFixedModel& goalModel();
  Leph::HumanoidFixedPressureModel& readModel();
  Leph::HumanoidFixedPressureModel& correctedModel();

  /**
   * Update given Model with past read state
   * interpolated at given timestamp.
   * The model does integrate corrected odometry.
   * The method is thread safe.
   * Also return the magnetometer absolute value
   * and boolean is base updated at given timestamp.
   */
  void pastReadModel(double timestamp,
                     Leph::HumanoidFixedPressureModel& pastReadModel);
  double pastMagneto(double timestamp);
  bool pastIsBaseUpdated(double timestamp);

  /**
   * Assign for the read model the base update option.
   * If true, the base position is integrated.
   */
  void setReadBaseUpdate(bool isEnable);

  /**
   * Returns true if _isUpdateReadBase was true atleast once since last call
   * (resets on call)
   */
  bool wasReadBaseUpdate();

  /**
   * Return the RhAL Timestamp and the flag
   * indicating if the last read have error values
   * associated to the Read Model.
   */
  const RhAL::TimePoint& readTimestamp();
  bool readIsError();

  /**
   * Compute and return [dx,dy,dtheta] relative
   * odometry displacement interpolated between given
   * timestamp in self frame of source position.
   */
  Eigen::Vector3d odometryDiff(
    double timestamp1, double timestamp2);

  /**
   * Start or stop the lowlevel logging.
   * For starting, a file path is given to dump
   * the data.
   * If isBinary is true, the log is written
   * in binary format
   */
  void startLogging(const std::string& filepath,
                    bool isBinary = true);
  void stopLogging();

  /**
   * Start a named log session on all histories
   */
  void startNamedLog(const std::string & filePath);
  /**
   * Stop adding new entries for the given log name (on all histories)
   * Then, write the logs, histories will be written at filePath
   */
  void stopNamedLog(const std::string & filePath);


  /**
   * Append to history container given name:value
   * couple to be logged at current timestamp
   */
  void logValue(const std::string& name, double value);

  /**
   * Enable the replay mode and load history
   * data from given log file name
   */
  void loadReplays(
    const std::string& filepath);

  /**
   * Assign the replay timestamp
   * for updating the read model
   * debug.
   */
  void setReplayTimestamp(double ts);

  /**
   * Access to Odometry displacement
   * model and utilities
   */
  Leph::Odometry& getOdometryModel();

  /**
   * Access to Camera Model angular parameters
   */
  const Leph::CameraParameters& getCameraParameters() const;

  /**
   * Override the FOV parameters (inputs in radians). Dirty hack to try to keep consistency between camera parameters...
   */
  void setCameraParametersFOV(const double fovx, const double fovy);

private:

  /**
   * Lowlevel missing state
   */
  std::string _lowlevelState;

  /**
   * If true, the IMU is disable for
   * pitch and roll (still use yaw).
   */
  bool _noIMU;

  /**
   * If true, the control and power
   * voltage of all motors are requested
   * and log in histories
   */
  bool _isReadVoltages;

  /**
   * Container for all read value hitories
   */
  std::map<std::string, rhoban_utils::History> _histories;

  /**
   * Model for target goal state
   */
  Leph::HumanoidFixedModel _goalModel;

  /**
   * Model for present read state
   */
  Leph::HumanoidFixedPressureModel _readModel;

  /**
   * Model for present state with
   * corrected odometry
   */
  Leph::HumanoidFixedPressureModel _correctedModel;

  /**
   * Camera Model camera angular parameters
   * and IMU angle offsets
   */
  Leph::CameraParameters _cameraParameters;
  Eigen::Vector3d _imuOffset;

  /**
   * If true, the read model base position is
   * integrated. Prevent odometry drfit when
   * the robot is stopped
   */
  bool _isUpdateReadBase;

  bool _wasUpdateReadBase;

  /**
   * If true, the read model update
   * is forced
   */
  bool _forceUpdateReadBase;

  /**
   * Timestamp associated to values used by
   * the Read Model
   */
  RhAL::TimePoint _timestamp;

  /**
   * If true, the replay mode is enable.
   * History values are not updated in step().
   * Current timestamp replaying.
   */
  bool _isReplay;
  double _replayTimestamp;

  /**
   * RhIO binding
   */
  RhIO::Bind _bind;

  /**
   * Current Model support foot
   */
  Leph::HumanoidFixedModel::SupportFoot _currentSupport;

  /**
   * Odometry displacement
   * correction model
   */
  Leph::Odometry _odometry;

  /**
   * If doWriteLog true, logs have to be
   * written in next tick().
   * Log filename and format option.
   */
  bool _doWriteLog;
  std::string _logPath;
  bool _isLogBinaryFormat;

  /**
   * Compute odometry displacement
   * between two given state from state1
   * to state2
   */
  Eigen::Vector3d odometryDiff(
    const Eigen::Vector3d& state1,
    const Eigen::Vector3d& state2) const;

  /**
   * Integrate given odometry diff to
   * given state and update it
   */
  void odometryInt(
    const Eigen::Vector3d& diff,
    Eigen::Vector3d& state) const;

  /**
   * Publish to RhIO given model state with given prefix
   */
  void publishModelState(
    const std::string& prefix,
    const Leph::HumanoidFixedModel& model,
    bool initialize = false);

  /**
   * Dump into file the contains of
   * histories data
   */
  void tickDumpLogs();

  /**
   * Check camera and lowlevel state
   * for teamplay error message
   */
  std::string getCameraState();
  void tickCheckLowlevelState();

  /**
   * Update the odometry from readModel
   * and save it into history
   */
  void tickOdometryUpdate();

  /**
   * Find the maximum motors timestamp
   */
  void tickFindTimestamp();

  /**
   * Assign motors state to readModel
   */
  void tickAssignReadDOF();

  /**
   * Assign pressure state to readModel
   */
  void tickAssignReadPressure();

  /**
   * Assign imu state to readModel
   */
  void tickAssignReadIMU();

  /**
   * Publish to RhIO odometry 
   * and models state
   */
  void tickRhIOPublish();

  /**
   * Push into hisotries read state
   */
  void tickHistoryUpdate();
};
