#pragma once

#include "SpecialImageHandler.hpp"

#include <thread>
#include "Filters/Pipeline.hpp"
#include "Filters/Custom/FieldBorderData.hpp"
#include "Application/Application.hpp"
#include "Utils/ImageLogger.h"

#include "rhoban_utils/timing/time_stamp.h"

#include <Eigen/Core>
#include <utility>
#include <string>
#include <vector>
#include <map>

class MoveScheduler;
namespace Vision {

class LogBenchmark;
class RobocupBenchmark;

namespace Localisation {
class BallStackFilter;
class RobotFilter;
class SpeedEstimator;
}

/**
 * Robocup
 *
 * Robocup vision pipeline struture
 * is defined here using VisionReloaded
 * framework
 */
class Robocup : public Vision::Application::Application {
  friend class Vision::RobocupBenchmark; // RobocupBenchmark need an access
  friend class Vision::LogBenchmark;     // LogBenchmark need an access
private:
  /**
   * Note on mutex:
   * - globalMutex:
   *   -> Required at any time when writing data
   *   -> Required when coherency is required between different information
   *sources
   * - csMutex:
   *   -> CameraState mutex, used for informations on timestamps and camera
   *state
   * - visionMutex:
   *   -> Used for pipelines and informations coming from vision
   *
   * WARNING:
   * - When a threads locks globalMutex, this thread cannot possess lock on any
   *other
   *   mutex.
   * - It is forbidden to lock 2 specific mutex without holding the globalMutex
   */
  mutable std::mutex globalMutex;
  mutable std::mutex visionMutex;

  mutable std::mutex logMutex;

  mutable std::mutex csMutex;

  int imageDelay; // Delay between the image capture time [ms]

  // Logging
  Utils::ImageLogger manual_logger;
  rhoban_utils::TimeStamp endLog;

  Utils::ImageLogger moving_ball_logger;
  /// If enabled each time a robot kicks the ball or ball is detected as moving,
  /// start a new log
  bool autologMovingBall;

  /// Keep logging for a while after ball stopped moving [s]
  float logBallExtraTime;

  /// If activated: write ball position and speed in self referntial at each
  /// vision step
  bool writeBallStatus;

  /// When was the ball moving for last time
  rhoban_utils::TimeStamp lastBallMoving;

  void initImageHandlers();

  void initObservationTypes();
  
  void initRhIO();
  void publishToRhIO();
  void importFromRhIO();

  MoveScheduler *_scheduler;

public:
  // Properties for monitoring images
  std::vector<SpecialImageHandler> imageHandlers;

  // Benchmark options
  bool benchmark;
  int benchmarkDetail;

  /**
   * Initialize and start
   * the Robocup pipeline
   */
  Robocup(MoveScheduler *scheduler);

  // Create a robocup config based on configFile
  // Required for config file
  Robocup(const std::string &configFile, MoveScheduler *scheduler);

  /**
   * Initialize and start
   * the Robocup pipeline with cmd-line args
   */
  Robocup(int argc, char **argv);

  /**
   * Stop the pipeline
   */
  virtual ~Robocup();

  void run();

  virtual void init() override;
  virtual void step() override;
  virtual void finish() override;

  void startLogging(unsigned int timeMS, const std::string & logDir);
  void endLogging();

  // How many frames were captured?
  int getFrames();

  // What is the current camera status?
  std::string getCameraStatus() const;
  double getLastUpdate() const;

  /// (x,y) is the position of the ball in robot referential [m]
  void ballReset(float x, float y);

  /// Clears the ball filter (no ball in it)
  void ballClear();
  void robotsClear();

  /// Asks the model to start a logging session with given path as a target
  void startLoggingLowLevel(const std::string & path);
  /// Asks the model to end and save a session with given path
  void stopLoggingLowLevel(const std::string & path);
  /// Tells the model to read the low level values from a log file instead than
  /// from the actual low level
  void setLogMode(const std::string path);

  void readPipeline();
  void getUpdatedCameraStateFromPipeline();
  void loggingStep();
  void updateBallInformations();

  /// Get all goals currently stored and remove them from the list
  std::vector<cv::Point2f> stealGoals();
  
  /// Lock mutex on tags, retrieve indices and position of tags
  /// Finally clear all memory about tags
  void stealTags(std::vector<int> & indices,
                 std::vector<Eigen::Vector3d> & positions,
                 std::vector<std::pair<float, float> > & centers,
                 std::vector<std::pair<float, float> > & undistorded_centers,
                 double * timestamp);


  //steal the observations from the visual compass
  void stealCompasses(std::vector<double> &orientations, std::vector<double> &dispersions);

  /// Get all clipping loc info currently stored and remove them from the list
  std::vector<Vision::Filters::FieldBorderData> stealClipping();

  // Apply a kick on the ball stack filter
  void applyKick(double x, double y);

  cv::Mat getRobotView(int width = 600, int height = 600);
  cv::Mat getTaggedImg();
  cv::Mat getTaggedImg(int width, int height);
  cv::Mat getRadarImg(int width, int height);

  cv::Mat getImg(const std::string &name, int wishedWidth, int wishedHeight,
                 bool gray);

  const Pipeline &getPipeline() const { return pipeline; }

  /* JSON STUFF */
  virtual Json::Value toJson() const override;
  virtual void fromJson(const Json::Value & v, const std::string & dir_name) override;
  virtual std::string getClassName() const override { return "vision_config"; }

  //TODO : move this into radar refactoring
  std::vector<cv::Point2f> keepFrontRobots(std::vector<cv::Point2f> & robots);
  
  void closeCamera();

  /// Uses MoveScheduler to check current mode (fake or active)
  bool isFakeMode() const;

  /// In classic mode, uses steady clock.
  /// In fake mode, uses pipeline clock.
  rhoban_utils::TimeStamp getNowTS() const;

  /**
   * Pipeline main loop thread
   */
  std::thread *_runThread;
  bool _doRun;

  // BALL
  /// Ball position filter
  Localisation::BallStackFilter * ballStackFilter;
  //  Opponent robots filter
  Localisation::RobotFilter * robotFilter;
  /// Ball speed estimator
  Localisation::SpeedEstimator * ballSpeedEstimator;

  // Sensors and related
  Utils::CameraState * cs;
  ::rhoban_utils::TimeStamp lastTS, sourceTS;

  bool ballDetected;
  std::vector<double> ballsX, ballsY, ballsRadius;

  // Estimating ball speed
  bool _firstLoop = true;

  // Connection status
  bool activeSource;

  /// When set to true, will remove all observations from the memory at next
  /// vision tick
  bool clearRememberObservations;

private:
  /// Detected positions for goals in "origin" basis
  std::vector<cv::Point2f> detectedGoals;

  /// Detected robots in "origin" basis
  std::vector<cv::Point2f> detectedRobots;
  
  std::vector<std::string> observationTypes;

  /// For each type of observation, the map contains a list of
  /// detected positions for the observation in "self" basis with a living time
  /// (can stay alive for more than 1 step)
  std::map<std::string, std::vector<std::pair<cv::Point2f, float>>> rememberObservations;

  /// Controls access to the goals
  mutable std::mutex goalsMutex;

  /// Controls access to the clipping
  mutable std::mutex clippingMutex;

  /// Indexes of the tags detected
  std::vector<int> detectedTagsIndices;
  /// Positions of the tags detected (in world frame)
  std::vector<Eigen::Vector3d> detectedTagsPositions;
  /// Positions of the center of the tags on the image (x, y), range [-1, 1]
  std::vector<std::pair<float, float> > detectedTagsCenters;
  /// Positions of the center of the tags on the undistort image (x, y), range [-1, 1]
  std::vector<std::pair<float, float> > detectedTagsCentersUndistort;
  /// timestamp of the tag detection
  double detectedTimestamp=0.0;

  /// Controls access to the tags
  mutable std::mutex tagsMutex;

  /// Controls access to the visualcompass
  mutable std::mutex compassMutex;

  //Orientations and corresponding dispersions (quality) for the visual compass
  std::vector<double> detectedOrientations;
  std::vector<double> detectedDispersions;

  std::vector<double> radarOrientations;
  std::vector<double> tmporientations;
  std::vector<double> tmpdispersions;

  /// key: featureName
  /// values: feature providers
  std::map<std::string,std::vector<std::string>> featureProviders;

  // clipping data for the localisation
  std::vector<Vision::Filters::FieldBorderData> clipping_data;
  
  /// Was robot handled at previous step
  bool wasHandled;

  /// Was robot fallen at previous step
  bool wasFallen;

  /// Are the ball outside of the field (according to localization) ignored
  bool ignoreOutOfFieldBalls;
};
}
