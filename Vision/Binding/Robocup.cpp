#include "Binding/Robocup.hpp"
#include <iostream>
#include <unistd.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "Localisation/Ball/BallStackFilter.hpp"
#include "Localisation/Ball/SpeedEstimator.hpp"

#include "Localisation/Robot/RobotFilter.hpp"

#include "scheduler/MoveScheduler.h"

#include "Filters/Ball/BallProvider.hpp"
#include "Filters/Features/CompassProvider.hpp"
#include "Filters/Features/VisualCompass.hpp"
#include "Filters/Custom/FieldBorderData.hpp"

#include "Filters/Features/TagsDetector.hpp"
#include "Filters/Goal/GoalProvider.hpp"
#include "Filters/Obstacles/ObstacleProvider.hpp"
#include "Filters/Source/SourcePtGrey.hpp"

#include "CameraState/CameraState.hpp"
#include "Utils/Drawing.hpp"
#include "Utils/Interface.h"

#include "rhoban_geometry/point.h"

#include "rhoban_utils/timing/benchmark.h"

#include "RhIO.hpp"
#include "robocup_referee/constants.h"

#include "moves/Head.h"
#include <rhoban_utils/logging/logger.h>
#include <rhoban_utils/util.h>

#include "Localisation/Field/CompassObservation.hpp"

#include "services/DecisionService.h"
#include "services/LocalisationService.h"
#include "services/ModelService.h"
#include "services/RefereeService.h"
#include <cmath>
#include <cstdlib>
#include <sstream>
#include <string>
#include <unistd.h>
#include "Filters/Custom/FieldBorder.hpp"

#include <vector>

#include <utility>
#include <algorithm>


static rhoban_utils::Logger out("vision_robocup");

using namespace Vision::Localisation;
using namespace rhoban_utils;
using namespace rhoban_geometry;

using namespace std::chrono;

using Vision::Utils::CameraState;
using Vision::Utils::ImageLogger;
using Vision::Filters::BallProvider;
using Vision::Filters::CompassProvider;
using Vision::Filters::TagsDetector;
using robocup_referee::Constants;

namespace Vision {

Robocup::Robocup(MoveScheduler *scheduler)
    : Application(),
      imageDelay(0),
      manual_logger("manual_logs", true, 10000),
      moving_ball_logger("moving_ball_logs", false, 1000),
      autologMovingBall(false),
      game_logger("game_logs", false, 15 * 60 * 40),// Allows 15 minutes at 40 fps ->
      autolog_games(false),
      logBallExtraTime(2.0),
      writeBallStatus(false),
      _scheduler(scheduler),
      benchmark(false), benchmarkDetail(0),
      cs(new CameraState(scheduler)), //TODO: maybe put the name of the param file elsewhere
      activeSource(false),
      clearRememberObservations(false),
      wasHandled(false),
      wasFallen(false),
      ignoreOutOfFieldBalls(true)
{
  ballStackFilter = new BallStackFilter(cs);
  robotFilter = new RobotFilter(cs);
  ballSpeedEstimator = new SpeedEstimator();
  featureProviders["arena"] = std::vector<std::string>();
  featureProviders["goal"] = std::vector<std::string>();
  featureProviders["ball"] = std::vector<std::string>();
  featureProviders["tags"] = std::vector<std::string>();
  featureProviders["compass"] = std::vector<std::string>();
  featureProviders["fieldBorder"] = std::vector<std::string>();
  initObservationTypes();

  for (std::string obs : observationTypes) {
    rememberObservations[obs] = std::vector<std::pair<cv::Point2f, float>>();
  }

  out.log( "Starting Robocup Pipeline");
  pipeline.setCameraState(cs);
  initImageHandlers();
  loadFile();
  _doRun = true;
  _runThread = new std::thread(std::bind(&Robocup::run, this));
  Filter::GPU_ON = gpuOn;
  if (pathToLog != "") {
    // The low level info will come from a log
    setLogMode(pathToLog);
  }
  scheduler->getServices()->localisation->setRobocup(this);
}

Robocup::Robocup(const std::string &configFile, MoveScheduler *scheduler)
    :  imageDelay(0),
       manual_logger("manual_logs", true, 10000),
       moving_ball_logger("moving_ball_logs", false, 1000),
       autologMovingBall(false),
       game_logger("game_logs", true, 15 * 60 * 40),// Allows 15 minutes at 40 fps
       autolog_games(false),
       logBallExtraTime(2.0),
       writeBallStatus(false),
       benchmark(false), benchmarkDetail(0),
       _runThread(NULL),
       cs(new CameraState(scheduler)),
       activeSource(false),
       clearRememberObservations(false),
       wasHandled(false),
       wasFallen(false),
      ignoreOutOfFieldBalls(true)
{

  ballStackFilter = new BallStackFilter(cs);
  robotFilter = new RobotFilter(cs);
  ballSpeedEstimator = new SpeedEstimator();
  featureProviders["arena"] = std::vector<std::string>();
  featureProviders["goal"] = std::vector<std::string>();
  featureProviders["ball"] = std::vector<std::string>();
  featureProviders["tags"] = std::vector<std::string>();
  featureProviders["compass"] = std::vector<std::string>();
  featureProviders["fieldBorder"] = std::vector<std::string>();
  initObservationTypes();

  for (std::string obs : observationTypes) {
    rememberObservations[obs] = std::vector<std::pair<cv::Point2f, float>>();
  }
  pipeline.setCameraState(cs);
  _scheduler = scheduler;
  initImageHandlers();
  loadFile(configFile);
  _doRun = true;
  Filter::GPU_ON = gpuOn;
  if (pathToLog != "") {
    // The low level info will come from a log
    setLogMode(pathToLog);
  }
  scheduler->getServices()->localisation->setRobocup(this);
}

Robocup::~Robocup() {
  delete ballStackFilter;
  _doRun = false;
  if (_runThread != NULL) {
    _runThread->join();
    delete _runThread;
  }
}

void Robocup::startLogging(unsigned int timeMS, const std::string &logDir) {
  // If logDir is empty a name session is generated automatically in manual_logger
  logMutex.lock();
  manual_logger.initSession(logDir);
  startLoggingLowLevel(manual_logger.getSessionPath() + "/lowLevel.log");
  endLog = TimeStamp(getNowTS() + milliseconds(timeMS));
  logMutex.unlock();
}

void Robocup::endLogging() {
  logMutex.lock();
  // Telling the low level to stop logging and to dump the info
  stopLoggingLowLevel(manual_logger.getSessionPath() + "/lowLevel.log");
  manual_logger.endSession();
  // TODO: examine if logMutex can be closed earlier 
  logMutex.unlock();
}

void Robocup::applyKick(double x, double y) {
  //ballStackFilter->applyKick(x / 100.0, y / 100.0);
}

cv::Mat Robocup::getImg(const std::string &name, int wishedWidth,
                        int wishedHeight, bool gray) {
  cv::Mat original, scaled, final;
  if (name == "Tagged") {
    original = getTaggedImg();
  } else {
    try {
      original = pipeline.get(name).getImg()->clone();
    } catch (const std::out_of_range &o) {
      throw std::runtime_error("Image not found : '" + name + "'");
    }
  }
  cv::resize(original, scaled, cv::Size(wishedWidth, wishedHeight));
  if (gray) {
    cv::cvtColor(scaled, final, CV_RGB2GRAY);
  } else {
    final = scaled;
  }
  return final;
}

Json::Value Robocup::toJson() const {
  // Writing stream
  Json::Value v = Application::toJson();
  v["benchmark"] = benchmark;
  v["benchmarkDetail"] = benchmarkDetail;
  v["imageDelay"] = imageDelay;
  v["autologMovingBall"] = autologMovingBall;
  v["autologGames"] = autolog_games;
  v["logBallExtraTime"] = logBallExtraTime;
  v["writeBallStatus"] = writeBallStatus;
  v["ignoreOutOfFieldBalls"] = ignoreOutOfFieldBalls;

  for (const auto &entry : featureProviders) {
    const std::string &featureName = entry.first;
    const std::vector<std::string> &providers = entry.second;
    v[featureName + "Providers"] = vector2Json(providers);
  }
  for (const SpecialImageHandler & sih : imageHandlers) {
    v[sih.name] = sih.display;
  }
  return v;
}

void Robocup::fromJson(const Json::Value & v, const std::string & dir_name) {
  Application::fromJson(v, dir_name);
  rhoban_utils::tryRead(v,"benchmark",&benchmark);
  rhoban_utils::tryRead(v,"benchmarkDetail",&benchmarkDetail);
  rhoban_utils::tryRead(v,"imageDelay",&imageDelay);
  rhoban_utils::tryRead(v,"autologMovingBall",&autologMovingBall);
  rhoban_utils::tryRead(v,"autologGames",&autolog_games);
  rhoban_utils::tryRead(v,"logBallExtraTime",&logBallExtraTime);
  rhoban_utils::tryRead(v,"writeBallStatus",&writeBallStatus);
  rhoban_utils::tryRead(v,"ignoreOutOfFieldBalls",&ignoreOutOfFieldBalls);
  for (auto &entry : featureProviders) {
    const std::string &featureName = entry.first;

    std::string nodeName = featureName + "Providers";
    rhoban_utils::tryReadVector<std::string>(v, nodeName, &entry.second);
  }
  for (SpecialImageHandler & sih : imageHandlers) {
    rhoban_utils::tryRead(v, sih.name, &sih.display);
  }
}

void Robocup::init() {
  Application::init();
  lastTS = ::rhoban_utils::TimeStamp::fromMS(0);

  initRhIO();
}

void Robocup::initImageHandlers() {
  imageHandlers.push_back(
      SpecialImageHandler("TaggedImg", 640, 480, [this](int width, int height) {
        return this->getTaggedImg(width, height);
      }));
  imageHandlers.push_back(
      SpecialImageHandler("RadarImg", 640, 480, [this](int width, int height) {
        return this->getRadarImg(width, height);
      }));
}

void Robocup::initRhIO() {
  // If a command has already been created do not pass here again
  if (RhIO::Root.commandExist("Vision/logLocal")) {
    return;
  }
  RhIO::Root.newStr("/Vision/cameraStatus")->defaultValue("");
  RhIO::Root.newFloat("/Vision/lastUpdate")
      ->defaultValue(-1)
      ->comment("Time since last update [ms]");
  // Init interface with RhIO
  if (isFakeMode()) {/// Highgui is not available on robot
    RhIO::Root.newCommand(
      "Vision/showFilters", "Display the given filters",
      [this](const std::vector<std::string> &args) -> std::string {
        if (args.size() < 1) {
          throw std::runtime_error("Usage: showFilters <name1> <name2> ...");
        }
        for (const std::string &name : args) {
          try {
            pipeline.get(name).display = true;
          } catch (const std::out_of_range &exc) {
            throw std::runtime_error("Filter " + name +
                                     " is not found in pipeline");
          }
        }
        return "Filters are now displayed";
      });
    RhIO::Root.newCommand(
      "Vision/hideFilters", "Hide the given filters",
      [this](const std::vector<std::string> &args) -> std::string {
        if (args.size() < 1) {
          throw std::runtime_error("Usage: hideFilters <name1> <name2> ...");
        }
        for (const std::string &name : args) {
          try {
            pipeline.get(name).display = false;
            cv::destroyWindow(name);
          } catch (const std::out_of_range &exc) {
            throw std::runtime_error("Filter " + name +
                                     " is not found in pipeline");
          }
        }
        return "Filters are now hidden";
      });
  }
  RhIO::Root.newCommand(
      "Vision/logLocal", "Starts logging for a specified duration. Images are "
                         "saved on board for now.",
      [this](const std::vector<std::string> &args) -> std::string {
        if (args.size() < 1) {
          throw std::runtime_error(
              "Usage: logLocal <duration[s]> <opt:log_dir>");
        }
        double duration = std::stof(args[0]);
        std::string logDir("");
        if (args.size() >= 2) {
          logDir = args[1];
        }
        this->startLogging((unsigned int)(duration * 1000), logDir);
        return "";
      });
  RhIO::Root.newBool("/Vision/autologMovingBall")
      ->defaultValue(autologMovingBall)
      ->comment("If enabled, start writing logs each time the ball is considered as moving");
  RhIO::Root.newFloat("/Vision/logBallExtraTime")
      ->defaultValue(logBallExtraTime)
      ->comment("Extra duration of log once ball stopped being flagged as moving [s]");
  RhIO::Root.newBool("/Vision/autologGames")
      ->defaultValue(autolog_games)
      ->comment("If enabled, write logs while game is playing");
  RhIO::Root.newBool("/Vision/benchmark")
      ->defaultValue(benchmark)
      ->comment("Is logging activated ?");
  RhIO::Root.newInt("/Vision/benchmarkDetail")
      ->defaultValue(benchmarkDetail)
      ->comment("Depth of print for benchmark");
  RhIO::Root.newInt("/Vision/imageDelay")
      ->defaultValue(imageDelay)
      ->maximum(200)
      ->minimum(-200)
      ->comment("Delay between sourceTS and reality");
  RhIO::Root.newFloat("/Vision/angularPitchTolerance")
      ->defaultValue(angularPitchTolerance)
      ->maximum(20.0)
      ->minimum(0.0)
      ->comment("Tolerance in pitch (degrees)");

  // Monitoring special images
  for (const SpecialImageHandler &sih : imageHandlers) {
    std::string prefix = "Vision/" + sih.name;
    RhIO::Root.newFloat(prefix + "_scale")->defaultValue(1.0)->comment("");
    RhIO::Root.newFrame(prefix, "", RhIO::FrameFormat::BGR);
  }

  ballStackFilter->bindToRhIO("ballStack", "ballStack");

  robotFilter->bindToRhIO("robotFilter", "robotFilter");
}

void Robocup::initObservationTypes() {
  observationTypes = {"ball", "post",   "team_mate", "opponent",
                      "t",    "corner", "tag", "clipping", "compass"};
}

void Robocup::finish() {
}

void Robocup::step() {
  if (clearRememberObservations) {
    for (auto & entry : rememberObservations) {
      entry.second.clear();
    }
    clearRememberObservations = false;
  }

  // Sometimes vision is useless and even bug prone, in this case, cancel the
  // step
  DecisionService *decision = _scheduler->getServices()->decision;
  bool handled = decision->handled;
  bool fallen = decision->isFallen;
  if (embedded && (handled || fallen)) {
    publishToRhIO();
    std::ostringstream oss;
    // Updating handled/fallen and sending message
    if (!wasHandled && handled) {
      out.log("Disabling vision (handled)");
      wasHandled = true;
    }
    if (wasHandled && !handled) {
      out.log("Robot is not handled anymore");
      wasHandled = false;
    }
    if (!wasFallen && fallen) {
      wasFallen = true;
      out.log("Disabling vision (fallen)");
    }
    if (wasFallen && !fallen) {
      out.log("Robot is not fallen anymore");
      wasFallen = false;
    }
    // Sleeping and waiting next step
    int ms_sleep = 10;
    usleep(ms_sleep * 1000);
    return;
  }
  if (wasHandled || wasFallen) {
    out.log("Starting to step vision again");
    wasHandled = false;
    wasFallen = false;
  }

  // If the camera is unplugged or doesn't respond for too long, we should avoid
  // keeping the filters data
  {
    double timeSinceLastFrame = diffSec(lastTS, getNowTS());
    if (timeSinceLastFrame > 5) {
      out.warning("no frame for %f, reseting the ball filter",
                  timeSinceLastFrame);

      // Resetting the ball stack filter
      ballStackFilter->clear();

      // Telling the localisation
      LocalisationService *loc = dynamic_cast<LocalisationService *>(
          _scheduler->getServices()->getService("localisation"));
      loc->setNoBall();
    }
  }

  // Making sure the image delay is given to the pipeline
  pipeline.imageDelay = imageDelay;
  cs->_angularPitchErrorDefault = angularPitchTolerance;
  importFromRhIO();
  Benchmark::open("Vision + Localisation");

  Benchmark::open("Waiting for global mutex");
  globalMutex.lock();
  Benchmark::close("Waiting for global mutex");

  Benchmark::open("Pipeline");
  try {
    Application::step();
    activeSource = true;
    // If Vision application has finished, ask for scheduler to shut down
    if (!isActive()) {
      out.log("Vision exiting, asking to scheduler to shut down");
      _scheduler->askQuit();
    }
  } catch (const Vision::Filters::PtGreyException &exc) {
    globalMutex.unlock();
    Benchmark::close("Pipeline");
    Benchmark::close("Vision + Localisation", benchmark, benchmarkDetail);
    activeSource = false;
    out.warning("Failed vision step: '%s'", exc.what());
    publishToRhIO();
    int sleep_time_ms = 100;
    usleep(sleep_time_ms * 1000);
    return;
  } catch (const Vision::Filters::PtGreyConnectionException &exc) {
    globalMutex.unlock();
    Benchmark::close("Pipeline");
    Benchmark::close("Vision + Localisation", benchmark, benchmarkDetail);
    activeSource = false;
    out.warning("Failed to connect to camera: '%s'", exc.what());
    publishToRhIO();
    int sleep_time_ms = 500;
    usleep(sleep_time_ms * 1000);
    return;
  }
  Benchmark::close("Pipeline");

  Benchmark::open("readPipeline");
  getUpdatedCameraStateFromPipeline();
  readPipeline();
  Benchmark::close("readPipeline");

  Benchmark::open("loggingStep");
  loggingStep();
  Benchmark::close("loggingStep");

  Benchmark::open("BallInformations");
  updateBallInformations();
  Benchmark::close("BallInformations");

  Benchmark::open("Tagging & Display");

  globalMutex.unlock();

  for (SpecialImageHandler &sih : imageHandlers) {
    std::string prefix = "Vision/" + sih.name;
    bool isStreaming = RhIO::Root.frameIsStreaming(prefix);
    // If frame is not displayed either streamed, avoid wasting CPU
    if ((!sih.display) && !isStreaming)
      continue;
    Benchmark::open(sih.name);
    // Update image and update necessary parts
    int img_width = sih.getWidth();
    int img_height = sih.getHeight();
    sih.lastImg = sih.getter(img_width, img_height);
    if (sih.display)
      cv::imshow(sih.name, sih.lastImg);
    if (isStreaming)
      RhIO::Root.framePush(prefix, img_width, img_height, sih.lastImg.data,
                           sih.getSize());
    Benchmark::close(sih.name.c_str());
  }

  Benchmark::open("Waiting for global mutex");
  globalMutex.lock();
  Benchmark::close("Waiting for global mutex");

  publishToRhIO();

  Benchmark::close("Tagging & Display");

  globalMutex.unlock();

  Benchmark::close("Vision + Localisation", benchmark, benchmarkDetail);

  // Set the log timestamp during fake mode
  if (isFakeMode()) {
    double ts = pipeline.getTimestamp().getTimeMS();
    _scheduler->getServices()->model->setReplayTimestamp(ts / 1000.0);
  }
}

void Robocup::importFromRhIO() {
  autologMovingBall = RhIO::Root.getValueBool("/Vision/autologMovingBall").value;
  autolog_games = RhIO::Root.getValueBool("/Vision/autologGames").value;
  logBallExtraTime = RhIO::Root.getValueFloat("/Vision/logBallExtraTime").value;
  benchmark = RhIO::Root.getValueBool("/Vision/benchmark").value;
  benchmarkDetail = RhIO::Root.getValueInt("/Vision/benchmarkDetail").value;
  imageDelay = RhIO::Root.getValueInt("/Vision/imageDelay").value;
  angularPitchTolerance =
      RhIO::Root.getValueFloat("/Vision/angularPitchTolerance").value;
  // Import size update for images
  for (SpecialImageHandler &sih : imageHandlers) {
    std::string prefix = "Vision/" + sih.name;
    sih.scale = RhIO::Root.getValueFloat(prefix + "_scale").value;
  }
}

void Robocup::publishToRhIO() {
  RhIO::Root.setFloat("/Vision/lastUpdate", diffMs(lastTS, getNowTS()));
  std::string cameraStatus = getCameraStatus();
  RhIO::Root.setStr("/Vision/cameraStatus", cameraStatus);
}

std::string Robocup::getCameraStatus() const {
  if (!activeSource) {
    return "Connection lost";
  }
  DecisionService *decision = _scheduler->getServices()->decision;
  if (decision->handled || decision->isFallen) {
    return "Inactive (handled or fallen)";
  }
  return "Active";
}

void Robocup::readPipeline() {
  std::vector<std::string> arenaProviders = featureProviders["arena"];
  std::vector<std::string> goalProviders = featureProviders["goal"];
  std::vector<std::string> ballProviders = featureProviders["ball"];
  std::vector<std::string> tagProviders = featureProviders["tags"];
  std::vector<std::string> fieldBorderProviders = featureProviders["fieldBorder"];
  std::vector<std::string> compassProviders = featureProviders["compass"];

  // Goals:
  goalsMutex.lock();
  for (const std::string &provider_name : goalProviders) {
    try {
      const Filters::GoalProvider &provider =
          dynamic_cast<const Filters::GoalProvider &>(
              pipeline.get(provider_name));
      std::vector<double> providerGoalsX = provider.getGoalsX();
      std::vector<double> providerGoalsY = provider.getGoalsY();
      // Converting the goal from image basis to "origin" basis
      // and adding it to detected goals
      for (size_t id = 0; id < providerGoalsX.size(); id++) {
        double goal_x = providerGoalsX[id];
        double goal_y = providerGoalsY[id];
        detectedGoals.push_back(cs->robotPosFromImg(
            goal_x, goal_y, 1, 1, false)); // false=invariant world reference
                                           // frame (which integrates the
                                           // odometry)
      }
    } catch (const std::bad_cast &e) {

      std::cerr
          << "Failed to import goal positions, check pipeline. Exception = "
          << e.what() << std::endl;
    } catch (const std::runtime_error &exc) {
      std::cerr << "Robocup::readPipeline: goalProviders: runtime_error: "
                << exc.what() << std::endl;
    }
  }
  goalsMutex.unlock();

  // Ball
  for (const std::string &providerName : ballProviders) {
    try {
      // Balls from BallProviders
      Vision::Filter &ballFilter = pipeline.get(providerName);
      const BallProvider &provider =
          dynamic_cast<const BallProvider &>(ballFilter);
      ballsX = provider.getBallsX();
      ballsY = provider.getBallsY();
      ballsRadius = provider.getBallsRadius();
    } catch (const std::bad_cast &e) {
      std::cerr
          << "Failed to import ball positions, check pipeline. Exception = "
          << e.what() << std::endl;
    }
  }

  // Robot detection
  if (pipeline.isFilterPresent("obstacleByDNN")) {
    try {
      Vision::Filter &robotDetector_f = pipeline.get("obstacleByDNN");
      const Filters::ObstacleProvider &robotDetector =
          dynamic_cast<const Filters::ObstacleProvider &>(robotDetector_f);

      const std::vector<Eigen::Vector2d> & frame_obstacles = robotDetector.getObstacles();

      if (robotFilter) {
        std::vector<Eigen::Vector3d> positions;
        for (const Eigen::Vector2d & obs : frame_obstacles) {
          auto tmp = cs->robotPosFromImg(obs.x(), obs.y(), 1, 1, false);
          Eigen::Vector3d in_world(tmp.x, tmp.y, 0);
          positions.push_back(in_world);
        }
        robotFilter->newFrame(positions);
  
        LocalisationService *loc = _scheduler->getServices()->localisation;
        std::vector<Eigen::Vector3d> filteredPositions;
        for (auto &candidate : robotFilter->getCandidates()) {
          // XXX: Threshold to Rhioize
          if (candidate.score > 0.45) {
            filteredPositions.push_back(candidate.object);
          }
        }
        loc->setOpponentsWorld(filteredPositions);
      }

      detectedRobots.clear();
      // Going from pixels to world referential
      for (const Eigen::Vector2d & obs : frame_obstacles) {
        detectedRobots.push_back(
          cs->robotPosFromImg(obs.x(), obs.y(), 1, 1, false));  // false=invariant world reference
                                                                // frame (which integrates the
                                                                // odometry)
      }
    } catch (const std::bad_cast &e) {
      std::cerr << "Failed to import robot positions, check pipeline. Exception = " << e.what() << std::endl;
    } catch (const std::runtime_error &exc) {
      std::cerr << "Robocup::readPipeline: robot detection: runtime_error: " << exc.what() << std::endl;
    }
  }

  // Tags
  for (const std::string & tagProviderName : tagProviders) {
    Vision::Filter &tagFilter = pipeline.get(tagProviderName);
    cv::Size size = pipeline.get(tagProviderName).getImg()->size();
    tagsMutex.lock();
    detectedTimestamp = pipeline.getTimestamp().getTimeMS() / 1000.0;
    try {
      const TagsDetector &tagProvider =
          dynamic_cast<const TagsDetector &>(tagFilter);
      std::vector<aruco::Marker> new_tags = tagProvider.getDetectedMarkers();
      for (const aruco::Marker &marker : new_tags) {
        // Adding Id
        detectedTagsIndices.push_back(marker.id);
        // Computing position of marker in self and adding it
        Eigen::Vector3d marker_pos_in_cam;
        // Changing camera basis from Usual notation to Leph::notation
        marker_pos_in_cam(0) = marker.Tvec.ptr<float>(0)[2];
        marker_pos_in_cam(1) = -marker.Tvec.ptr<float>(0)[0];
        marker_pos_in_cam(2) = -marker.Tvec.ptr<float>(0)[1];
        Eigen::Vector3d marker_pos_in_self =
            cs->_model->frameInSelf("camera", marker_pos_in_cam);
        Eigen::Vector3d marker_pos_in_world =
            cs->_model->selfInFrame("origin", marker_pos_in_self);
        // DEBUG
        // std::cout << "Marker pos in cam   : " <<
        // marker_pos_in_cam.transpose()    << std::endl;
        // std::cout << "Marker pos in self  : " <<
        // marker_pos_in_self.transpose()   << std::endl;
        // std::cout << "Marker pos in world : " <<
        // marker_pos_in_world.transpose()  << std::endl;

        // Adding position of marker in world
        detectedTagsPositions.push_back(marker_pos_in_world);

        // Calculating the center of the tags on the image (x, y)
        // TODO, if the barycenter is not good enough, do better (crossing the
        // diags?)
        Eigen::Vector2d avg_in_img(0,0);
        for (unsigned int i = 0; i < marker.size(); i++) {
          avg_in_img += Eigen::Vector2d(marker[i].x, marker[i].y);
        }
        avg_in_img /= 4.0;
        // Undistort position
        cv::Point2f avg_undistorded;
        cs->undistortPoint(avg_in_img(0), avg_in_img(1), size.width, size.height, avg_undistorded);
        // Using a pair instead than a cv::Point so the structure is usable even
        // without opencv (will be read by the low level)
        std::pair<float, float> pair_in_img(avg_in_img(0) / size.width,
                                            avg_in_img(1) / size.height);
        std::pair<float, float> pair_undistorded(avg_undistorded.x / size.width,
                                                 avg_undistorded.y / size.height);
        detectedTagsCenters.push_back(pair_in_img);
        detectedTagsCentersUndistort.push_back(pair_undistorded);
      }
    } catch (const std::bad_cast &exc) {
      tagsMutex.unlock();
      throw std::runtime_error("Invalid type for filter 'TagsDetector'");
    } catch (...) {
      tagsMutex.unlock();
      throw;
    }
    tagsMutex.unlock();
  }

  // Clipping and line detection
  // Ajout des lignes du clipping
  clippingMutex.lock();
  /*
  for (const std::string & name : fieldBorderProviders) {
    Vision::Filter & fieldBorder_f = pipeline.get(name);
    try {
      const Filters::FieldBorder & fieldBorder =
        dynamic_cast<const Filters::FieldBorder &>(fieldBorder_f);
      Filters::FieldBorderData data = fieldBorder.loc_data;
      clipping_data.push_back(data);
    } catch (const std::bad_cast & e) {
      out.log("Failed to cast filter '%s' in FieldBorder", name.c_str());
    }
  }
  */
  // TEMP TEMP

  if (pipeline.isFilterPresent("fieldBorder")) {
    Vision::Filter & fieldBorder_f = pipeline.get("fieldBorder");
    try {
      const Filters::FieldBorder & fieldBorder =
        dynamic_cast<const Filters::FieldBorder &>(fieldBorder_f);
      Filters::FieldBorderData data = fieldBorder.loc_data;
      clipping_data.push_back(data);
    } catch (const std::bad_cast & e) {
      //out.log("Failed to cast filter '%s' in FieldBorder", name.c_str());
      out.log("Failed to cast filter in FieldBorder");
    }
  }
  // /TEMP /TEMP
  clippingMutex.unlock();
  
  // VisualCompass
  compassMutex.lock();
  if (compassProviders.size() > 1) {
    //TODO: fill vector properly in order to support multiple compassProviders
    throw std::logic_error("Robocup: Only 1 compassProvider is supported now");
  }
  for (const std::string & name : compassProviders) {
    Vision::Filter & compassFilter = pipeline.get(name);
    try {
      const Vision::Filters::VisualCompass & CompassProvider =
        dynamic_cast<const Vision::Filters::VisualCompass &>(compassFilter);
      tmporientations=CompassProvider.getCompasses();
      tmpdispersions=CompassProvider.getDispersions();
      radarOrientations.clear();
      // orientations.reserve(orientations.size()+tmporientations.size());
      // orientations.insert(orientations.end(),tmporientations.begin(),tmporientations.end());

      detectedDispersions.reserve(detectedDispersions.size()+tmpdispersions.size());
      detectedDispersions.insert(detectedDispersions.end(),tmpdispersions.begin(),tmpdispersions.end());


      for (double angle : tmporientations) {
        Angle yaw=cs->getYaw(); //yaw of the camera
        Angle trunkYaw=cs->getTrunkYawInWorld(); //yaw of the trunk in the world
        double opGoalCapOffset=0.0; //TODO
        //angle from the visualcompass is in the anti-normal sign
        //TODO
        double dirGoalInCamera=rad2deg(angle)-opGoalCapOffset;
        double dirGoalInTrunk=dirGoalInCamera+yaw.getSignedValue();
        double dirGoalInWorld=dirGoalInTrunk+trunkYaw.getSignedValue();// Validated until here

        // Compass value for the trunk in world basis
        detectedOrientations.push_back(dirGoalInWorld);
        radarOrientations.push_back(dirGoalInWorld);
      }

    } catch (const std::bad_cast &e) {

      std::cerr
        << "Failed to import visual compass stuff, check pipeline. Exception = "
        << e.what() << std::endl;
    }
  }
  compassMutex.unlock();
}

void Robocup::getUpdatedCameraStateFromPipeline() {
  csMutex.lock();

  // Backup values
  lastTS = sourceTS;

  sourceTS = cs->getTimeStamp();

  // TODO: identify if this part is only debug
  if (!isFakeMode()) {
    double timeSinceLastFrame = diffSec(lastTS, sourceTS);
    if (timeSinceLastFrame > 2 || timeSinceLastFrame < 0) {
      out.warning("Suspicious elapsed time: %f [s]", timeSinceLastFrame);
    }
  }

  csMutex.unlock();
}

void Robocup::loggingStep() {
  TimeStamp now = getNowTS();

  DecisionService * decision = _scheduler->getServices()->decision;
  RefereeService * referee = _scheduler->getServices()->referee;

  // Capture src_filter and throws a std::runtime_error if required
  const Filter & src_filter =  pipeline.get("source");
  ImageLogger::Entry entry(pipeline.getTimestamp(), *(src_filter.getImg()));

  logMutex.lock();
  // Handling manual logs
  bool dumpManualLogs = false;
  if (manual_logger.isActive()) {
    if (endLog < now) {// If time is elapsed: close log
      dumpManualLogs = true;
    } else {// If there is still time left, add entry
      try {
        manual_logger.pushEntry(entry);
      } catch (const ImageLogger::SizeLimitException & exc) {
        out.warning("Automatically stopping manual log because size limit was reached");
        dumpManualLogs = true;
      }
    }
  }
  // Handling moving ball logs
  if (decision->isBallMoving) {
    lastBallMoving = now;
  }
  double elapsedSinceBallMoving = diffSec(lastBallMoving, now);
  // Status of autoLog
  bool autoLogActive = moving_ball_logger.isActive();
  bool startAutoLog = autologMovingBall && !autoLogActive &&  decision->isMateKicking;
  bool stopAutoLog = autoLogActive &&
    !(elapsedSinceBallMoving < logBallExtraTime || decision->isMateKicking);
  bool useAutoLogEntry = startAutoLog || (autoLogActive && !stopAutoLog);
  // Starting autoLog
  if (startAutoLog) {
    moving_ball_logger.initSession();
    out.log("Starting a session at '%s'", moving_ball_logger.getSessionPath().c_str());
    std::string lowLevelPath = moving_ball_logger.getSessionPath() + "/lowLevel.log";
    startLoggingLowLevel(lowLevelPath);
  }
  // Trying to log entry (can fail is maxSize is reached)
  if (useAutoLogEntry) {
    try {
      moving_ball_logger.pushEntry(entry);
    } catch (const ImageLogger::SizeLimitException & exc) {
      stopAutoLog = true;
    }
  }

  // Status of game_logs
  bool is_playing = referee->isPlaying();
  bool gameLogActive = game_logger.isActive();
  bool useGameLogEntry = autolog_games && is_playing;
  bool startGameLog = !gameLogActive && useGameLogEntry;
  bool stopGameLog = gameLogActive && !useGameLogEntry;
  if (startGameLog) {
    game_logger.initSession();
    out.log("Starting a session at '%s'", game_logger.getSessionPath().c_str());
    std::string lowLevelPath = game_logger.getSessionPath() + "/lowLevel.log";
    startLoggingLowLevel(lowLevelPath);
  }
  if (useGameLogEntry) {
    try {
      game_logger.pushEntry(entry);
    } catch (const ImageLogger::SizeLimitException & exc) {
      stopGameLog = true;
    }
  }
  logMutex.unlock();

  // Writing logs is delayed until logMutex has been unlocked to avoid
  // unnecessary lock of ressources
  if (dumpManualLogs) {
    _scheduler->stopMove("head",0.5);
    endLogging();
  }
  if (stopAutoLog) {
    std::string lowLevelPath = moving_ball_logger.getSessionPath() + "/lowLevel.log";
    stopLoggingLowLevel(lowLevelPath);
    moving_ball_logger.endSession();
  }
  if (stopGameLog) {
    std::string lowLevelPath = game_logger.getSessionPath() + "/lowLevel.log";
    stopLoggingLowLevel(lowLevelPath);
    game_logger.endSession();
  }
}

void Robocup::updateBallInformations() {
  // Getting candidates in ball by ROI
  std::vector<Eigen::Vector3d> positions;
  for (size_t k = 0; k < ballsX.size(); k++) {
    try {
      if (ignoreOutOfFieldBalls) {
        cv::Point2f posInSelf = cs->robotPosFromImg(ballsX[k], ballsY[k], 1, 1);
        double ballXSelf = posInSelf.x;
        double ballYSelf = posInSelf.y;
        LocalisationService * localisation = _scheduler->getServices()->localisation;
        rhoban_geometry::Point robot = localisation->getFieldPos();
        Angle robotDir(rad2deg(localisation->getFieldOrientation()));
        double ballXField = robot.x + cos(robotDir) * ballXSelf - sin(robotDir) * ballYSelf;
        double ballYField = robot.y + sin(robotDir) * ballXSelf + cos(robotDir) * ballYSelf;
        // OPTION: Margin could be added here
        if (std::fabs(ballXField) > Constants::field.fieldLength/2 + Constants::field.borderStripWidth ||
            std::fabs(ballYField) > Constants::field.fieldWidth/2 + Constants::field.borderStripWidth) {
          out.warning("Ignoring a ball candidate outside of the field at (%f,%f)",
                      ballXField, ballYField);
          continue;
        }
      }
      // Usual code
      cv::Point2f ballPix(ballsX[k], ballsY[k]);
      Eigen::Vector3d ball_in_world = cs->ballInfoFromPixel(ballPix, 1, 1);
      positions.push_back(ball_in_world);
      ballSpeedEstimator->update(cs->getTimeStamp(),
                                 Eigen::Vector2d(ball_in_world(0), ball_in_world(1)));
    } catch (const std::runtime_error &exc) {
      out.warning("Ignoring a candidate at (%f,%f) because of '%s'",
                  ballsX[k], ballsY[k], exc.what());
    }
  }
  // Positions are transmitted in the world referential
  ballStackFilter->newFrame(positions);

  // Broadcast information to localisation service
  // Sending data to the loc
  LocalisationService *loc = _scheduler->getServices()->localisation;

  if (ballStackFilter->getCandidates().size() > 0) {
    auto lookCandidate = ballStackFilter->getCandidates().back();
    auto bestCandidate = ballStackFilter->getBest();
    double bsfMaxScore = ballStackFilter->getMaximumScore();
    Point ballSpeed = ballSpeedEstimator->getUsableSpeed();
    loc->setBallWorld(bestCandidate.object, lookCandidate.object,
                      bestCandidate.score / bsfMaxScore, ballSpeed,
                      cs->getTimeStamp());
  } else {
    loc->setNoBall();
  }

  /// If active: write ballStatus
  if (writeBallStatus) {
    // Some properties are shared for the frame
    double time = getNowTS().getTimeSec();
    for (const Eigen::Vector3d & pos_in_world : positions) {
      Point ball_pos_in_field = loc->worldToField(pos_in_world);
      Point robot_pos = loc->getFieldPos();
      double field_dir = normalizeRad(loc->getFieldOrientation());
      
      Eigen::Vector2d tmp = pos_in_world.segment(0,2);
      cv::Point2f pos_in_self = cs->getPosInSelf(rg2cv2f(tmp));
      // Entry format:
      // TimeStamp, ballX, ballY, robotX, robotY, fieldDir
      out.log("ballStatusEntry: %lf,%f,%f,%f,%f,%f",
              time, ball_pos_in_field.x, ball_pos_in_field.y,
              robot_pos.x, robot_pos.y, field_dir);
    }
  }
}

std::vector<cv::Point2f> Robocup::stealGoals() {
  goalsMutex.lock();
  std::vector<cv::Point2f> goalsCopy = detectedGoals;
  detectedGoals.clear();
  goalsMutex.unlock();
  return goalsCopy;
}

std::vector<Filters::FieldBorderData> Robocup::stealClipping() {
  clippingMutex.lock();
  std::vector<Filters::FieldBorderData> clippingCopy = clipping_data;
  clipping_data.clear();
  clippingMutex.unlock();
  return clippingCopy;
}
  
void Robocup::stealTags(std::vector<int> &indices,
                        std::vector<Eigen::Vector3d> &positions,
                        std::vector<std::pair<float, float>> &centers,
                        std::vector<std::pair<float, float>> &undistorded_centers,
                        double *timestamp) {
  tagsMutex.lock();
  indices = detectedTagsIndices;
  positions = detectedTagsPositions;
  centers = detectedTagsCenters;
  undistorded_centers = detectedTagsCentersUndistort;
  *timestamp = detectedTimestamp;
  detectedTagsIndices.clear();
  detectedTagsPositions.clear();
  detectedTagsCenters.clear();
  detectedTagsCentersUndistort.clear();
  tagsMutex.unlock();
}

void Robocup::stealCompasses(std::vector<double> &orientations, std::vector<double> &dispersions)
{
  compassMutex.lock();
  orientations = detectedOrientations;
  dispersions = detectedDispersions;
  detectedOrientations.clear();
  detectedDispersions.clear();
  compassMutex.unlock();
}


cv::Mat Robocup::getTaggedImg() {
  cv::Size size = pipeline.get("source").getImg()->size();
  return getTaggedImg(size.width, size.height);
}

/// Black:
/// - ArenaBorders
/// - Goals by ROI (classic)
/// - Number of the ball stack
/// Red:
/// - Direction to opponent goal according to the particle filter
///   - Thick line: direction to the goal
///   - Thin line: direction to the post
/// - Goals By ROI with Tocard detection
/// Blue:
/// - Goals by ROI with standard clipping
/// - The ball detcted
/// - Horizon
/// Cyan:
/// - Position of the ball inside the stack
/// Magenta:
/// - Detected robots
cv::Mat Robocup::getTaggedImg(int width, int height) {
  if (cs == NULL)
    throw std::runtime_error("TaggedImg not ready");
  cv::Mat tmp, img, tmp_small;
  globalMutex.lock();

  tmp = pipeline.get("source").getImg()->clone();

  cv::resize(tmp, tmp_small, cv::Size(width, height));

  cv::cvtColor(tmp_small, img, CV_YCrCb2BGR);

  // Tagging horizon
  double halfTagAngle =
      (cs->getVertApertureDeg() + cs->getLatApertureDeg()) / 2;
  if (halfTagAngle > 180)
    halfTagAngle = 180; // Handling wide angle camera

  {
      // Drawing candidates of balls
      auto candidates = ballStackFilter->getCandidates();
      int k = 0;
      for (auto &candidate : candidates) {
        k++;
        auto cpos = candidate.object;
        auto pos = cs->imgXYFromRobotPosition(cv::Point2f(cpos.x(), cpos.y()),
                                              img.cols, img.rows, false);
        // I candidate is outside of the image, ignore it
        if (pos.x < 0 && pos.y < 0) continue;

        // Draw ball candidate
        cv::circle(img, pos, 6*candidate.score, cv::Scalar(255, 255, 0), CV_FILLED);

        // Write candidate number
        std::stringstream ss;
        ss << (candidates.size() - k);
        cv::putText(img, ss.str(), pos, cv::FONT_HERSHEY_SIMPLEX, 0.3,
                    cv::Scalar(0, 0, 0), 1);

        // # Futur position of ball
        // Elapsed time
        double tag_ball_anticipation = 0.2;// [s] (TODO: set as rhio parameters?)
        double elapsed = tag_ball_anticipation;
        elapsed += diffSec(cs->getTimeStamp(), getNowTS());
        // Compute futur position
        Point ball_usable_speed = ballSpeedEstimator->getUsableSpeed();
        Eigen::Vector3d ball_speed(ball_usable_speed.x, ball_usable_speed.y, 0);

        auto next_cpos = cpos +  ball_speed * elapsed;
        auto futur_pos = cs->imgXYFromRobotPosition(cv::Point2f(next_cpos.x(), next_cpos.y()),
                                                    img.cols, img.rows, false);
        // Avoid drawing line if out of image
        if (futur_pos.x < 0 && futur_pos.y < 0) continue;
        cv::line(img, pos, futur_pos, cv::Scalar(255, 255, 0), 2);
      }
  }
  
  {
      // Drawing candidates of robots
      auto candidates = robotFilter->getCandidates();
      int k = 0;
      for (auto &candidate : candidates) {
        k++;
        auto cpos = candidate.object;
        auto pos = cs->imgXYFromRobotPosition(cv::Point2f(cpos.x(), cpos.y()),
                                              img.cols, img.rows, false);

        // I candidate is outside of the image, ignore it
        if (pos.x < 0 && pos.y < 0) continue;

        // Draw robot candidate
        cv::circle(img, pos, 6*candidate.score, cv::Scalar(255, 0, 255), CV_FILLED);

        // Write candidate number
        std::stringstream ss;
        ss << (candidates.size() - k);
        cv::putText(img, ss.str(), pos, cv::FONT_HERSHEY_SIMPLEX, 0.3,
                    cv::Scalar(0, 0, 0), 1);
      }
  }

  // Tagging balls seen on current image
  for (unsigned int ballIndex = 0; ballIndex < ballsRadius.size();
       ballIndex++) {
    if (ballsRadius[ballIndex] <= 0)
      continue;
    cv::Point center(ballsX[ballIndex] * img.cols,
                     ballsY[ballIndex] * img.rows);
    cv::circle(img, center, ballsRadius[ballIndex] * img.cols,
               cv::Scalar(100, 0, 100), 2);
    int radiusMin = 0.0;
    int radiusMax = 0.0;
    cs->ballInfoFromPixel(center, img.cols, img.rows, &radiusMin, &radiusMax);
    cv::circle(img, center, radiusMin, cv::Scalar(100, 200, 100), 1);
    cv::circle(img, center, radiusMax, cv::Scalar(100, 200, 100), 1);
  }

  // Drawing opponent robots
  auto candidates = ballStackFilter->getCandidates();

  // Tagging horizon
  cv::Point horizon1, horizon2;
  horizon1.x = 0;
  horizon2.x = width - 1;
  horizon1.y = cs->getPixelYtAtHorizon(horizon1.x, width, height);
  horizon2.y = cs->getPixelYtAtHorizon(horizon2.x, width, height);
  // Color in BGR
  cv::line(img, horizon1, horizon2, cv::Scalar(255, 0, 0), 2);

  globalMutex.unlock();

  return img;
}

cv::Mat Robocup::getRadarImg(int width, int height) {
  if (cs == NULL)
    throw std::runtime_error("RadarImg not ready");
  cv::Mat img;
  img = cv::Mat(height, width, CV_8UC3, cv::Scalar(0, 30, 0));
  // Drawing the robot
  cv::circle(img, cv::Point2i(width / 2, height / 2), 5,
             cv::Scalar(0, 200, 200), -1);
  globalMutex.lock();

  std::vector<cv::Point2f> freshObservations;
  std::vector<int> delete_me;
  // scale_factor -> conversion [m] -> [px]
  float scale_factor = width / (2 * Constants::field.fieldLength);
  int ball_radius = 5;//px
  cv::Scalar ball_color = cv::Scalar(0, 0, 200);
  int goal_size = 8;//px
  cv::Scalar goal_color = cv::Scalar(200, 0, 0);
  //int robot_size = 20;
  //cv::Scalar robot_color = cv::Scalar(255, 0, 255);
  float discount = 0.05;
  float merge_dist = 0.0; // in meters. Disabled by default. TODO make this an
                          // angular condition instead

  // Drawing satic distance marquers each meter (light circles are 0.5 meters)
  for (int i = 1; i < (1 + 2 * Constants::field.fieldLength); i++) {
    cv::circle(img, cv::Point2i(width / 2, height / 2),
               (i / 2.0) * scale_factor, cv::Scalar(0, 150, 0), 2 - (i % 2));
  }
  // Drawing vision cone
  double yaw = -cs->getYaw().getSignedValue() * M_PI / 180.0;
  double half_aperture = cs->getLatApertureDeg() * M_PI / (2 * 180.0);
  cv::Point2i p1(width / 2 + height * cos(yaw + half_aperture),
                 height / 2 + height * sin(yaw + half_aperture));
  cv::Point2i p2(width / 2 + height * cos(yaw - half_aperture),
                 height / 2 + height * sin(yaw - half_aperture));

  cv::line(img, cv::Point2i(width / 2, height / 2), p1, cv::Scalar(0, 100, 100),
           2);
  cv::line(img, cv::Point2i(width / 2, height / 2), p2, cv::Scalar(0, 100, 100),
           2);
  // 0° orientation (front)
  cv::line(img, cv::Point2i(width / 2, height / 2),
           cv::Point2f(width, height / 2), cv::Scalar(255, 0, 0), 1);

  for (std::string obsType : observationTypes) {
    std::vector<std::pair<cv::Point2f, float>> &storedObservations =
        rememberObservations[obsType];
    // Discounting and Killing observations
    delete_me.clear();
    freshObservations.clear();
    for (unsigned int i = 0; i < storedObservations.size(); i++) {
      auto scored_obs = storedObservations[i];
      // Discounting all the old observations
      storedObservations[i].second = storedObservations[i].second - discount;
      if (scored_obs.second < 0) {
        // This observation is considered dead now (checking this only once)
        delete_me.push_back(i);
      }
    }


    for (int i = delete_me.size() - 1; i > -1; i--) {
      // Erasing from the end of the vector to the start, so the smaller indexes
      // don't change.
      storedObservations.erase(storedObservations.begin() + delete_me[i]);
    }

    // Reading the fresh observations (observation type dependent)
    if (obsType == "ball") {
      for (unsigned int ballIndex = 0; ballIndex < ballsX.size(); ballIndex++) {
        // Going from and image position to a position on the field, in the
        // origin (of world) frame
        try {
          auto point = cs->robotPosFromImg(ballsX[ballIndex], ballsY[ballIndex],
                                           1, 1, false);
          // std::cout << "DEBUG Ball seen at (origin frame) " << point << std::endl;
          freshObservations.push_back(point);
        } catch (const std::runtime_error &exc) {
          // Ignore the candidate
        }
      }
    } else if (obsType == "post") {
      // Reading posts
      cv::Point2f goal;
      for (unsigned int index = 0; index < detectedGoals.size(); index++) {
        goal = detectedGoals[index];
        // std::cout << "Goal seen at (origin frame) " << goal << std::endl;
        freshObservations.push_back(goal);
      }
    } else if (obsType == "tag") {
      // Reading tags
      // Going from and image position to a position on the field, in the origin
      // (of world) frame
      for (unsigned int index = 0; index < detectedTagsCenters.size();
           index++) {
        try {
          std::cout << "DetectedTagsCenter at "
                    << detectedTagsCenters[index].first << ", "
                    << detectedTagsCenters[index].second << std::endl;
          // Going from and image position to a position on the field, in the
          // origin (of world) frame
          auto point = cs->robotPosFromImg(detectedTagsCenters[index].first,
                                           detectedTagsCenters[index].second, 1,
                                           1, false);
          freshObservations.push_back(point);
        } catch (const std::runtime_error &exc) {
          // Ignore the candidate
        }
      }
    } else if (obsType == "compass") {
      cv::Point2f opponentDir;
      for (unsigned int index = 0; index < radarOrientations.size(); index++) {
        // we got a direction => transforming into a point
        // x: angle in degree
        // y: quality in [0,1]
        opponentDir.x = radarOrientations[index];
        opponentDir.y = tmpdispersions[index];
        freshObservations.push_back(opponentDir);
      }
    } else if (obsType == "clipping") {
      //From position in world to robot relative position
      for (unsigned int index=0; index < clipping_data.size(); index++) {
	// ** CAUTION : HACK ** ... une observation peut être
	// - plusieurs lignes
	// - un coin ou pas
	// le premier élément est une entete:
	// x contient le nombre de ligne et y s'il y a un coin (1) ou pas.
	// Ensuite, chaque ligne est représenté par 2 élements et
	// le coin optionel 1 élément.
        if (clipping_data[index].is_obs_valid()) {
          int has_coin = clipping_data[index].hasCorner ? 1 : 0;
          int has_segment = clipping_data[index].hasSegment ? 1 : 0;
          std::vector<std::pair<cv::Point2f, cv::Point2f > > lines;
          if (has_coin) {
            lines = clipping_data[index].getLinesInWorldFrame();
          }
          if (has_segment) {
            lines.push_back(clipping_data[index].getSegmentInWorld());
          }
          int line_nb = lines.size();
          // int line_nb = clipping_data[index].getLinesInWorldFrame().size();
          freshObservations.push_back(cv::Point2f(line_nb, has_coin));
          for(auto L : lines) {
            freshObservations.push_back(L.first);
            freshObservations.push_back(L.second);
            // printf("Find line (%f,%f)->(%f,%f)\n",
            //       L.first.x, L.first.y, L.second.x, L.second.y);
          }
          if (clipping_data[index].hasCorner) {
            freshObservations.push_back(clipping_data[index].getCornerInWorldFrame());
            // printf("Find corner %f %f\n", clipping_data[index].getCornerInWorldFrame().x, clipping_data[index].getCornerInWorldFrame().y);
          }
        }
      }
    } else if (obsType == "opponent") {
      // Reading posts
      cv::Point2f robot;
      for (unsigned int index = 0; index < detectedRobots.size(); index++) {
        robot = detectedRobots[index];
        std::cout << "New robot seen at (origin frame) " << robot << std::endl;
        freshObservations.push_back(robot);
      } 
    } else {
      // Unhandled observation type
      continue;
    }

    // Adding the observations to the old ones if need be, and updating the
    // intensities
    for (unsigned int j = 0; j < freshObservations.size(); j++) {
      bool found = false;
      cv::Point2f new_obs = freshObservations[j];
      if (obsType != "clipping") { // Pas d'identification pour le clipping
	for (unsigned int i = 0; i < storedObservations.size(); i++) {
	  auto scored_obs = storedObservations[i];
	  float dist = sqrt((new_obs.x - scored_obs.first.x) *
			    (new_obs.x - scored_obs.first.x) +
			    (new_obs.y - scored_obs.first.y) *
			    (new_obs.y - scored_obs.first.y));
	  if (dist < merge_dist) {
	    // We're assuming it's the same observation
	    storedObservations[i].second = 1.0;
	    found = true;
	    // Can't be merged to more than one old observation (even if the dist
	    // condition could be valid for more than 1 old observation)
	    break;
	  }
	}
      }
      if (!found) {
        // Adding the observation
        storedObservations.push_back(std::pair<cv::Point2f, float>(new_obs, 1.0));
      }
    }

      // for (unsigned int i=0; i < storedObservations.size(); i++) {
      //   std::cout << obsType <<" at " << storedObservations[i].first << " score "
      //             << storedObservations[i].second << std::endl;
      // }
    // Drawing
    if (obsType == "opponent") {
      for (unsigned int i = 0; i < storedObservations.size(); i++) {
        // Going from meters to pixels, and from the origin frame to the robot
        // one
        cv::Point2f robot_in_self = cs->getPosInSelf(storedObservations[i].first);
        robot_in_self.x = max(0, (int)(robot_in_self.x * scale_factor + width / 2));
        robot_in_self.y = max(0, (int)(-robot_in_self.y * scale_factor + height / 2));

        cv::circle(img, robot_in_self, 15, cv::Scalar(200, 0, 200), -1);
      }
    } else if (obsType == "ball") {
      for (unsigned int i = 0; i < storedObservations.size(); i++) {
        // Going from meters to pixels, and from the origin frame to the robot
        // one
        cv::Point2f ball_in_self =
          cs->getPosInSelf(storedObservations[i].first);
        ball_in_self.x =
          max(0, (int)(ball_in_self.x * scale_factor + width / 2));
        ball_in_self.y =
          max(0, (int)(-ball_in_self.y * scale_factor + height / 2));

        cv::circle(img, ball_in_self, ball_radius, ball_color, -1);
      }


    } else if (obsType == "post") {
      for (unsigned int i = 0; i < storedObservations.size(); i++) {
        // Going from meters to pixels, and from the origin frame to the robot
        // one
        cv::Point2f goal_in_self =
          cs->getPosInSelf(storedObservations[i].first);
        goal_in_self.x =
          max(0, (int)(goal_in_self.x * scale_factor + width / 2));
        goal_in_self.y =
          max(0, (int)(-goal_in_self.y * scale_factor + height / 2));

        cv::Point2i second_point(goal_in_self.x + goal_size,
                                 goal_in_self.y + goal_size / 2);
        cv::rectangle(img, goal_in_self, second_point, goal_color, -1);
      }
    } else if (obsType == "tag") {
      for (unsigned int i = 0; i < storedObservations.size(); i++) {
        // Going from meters to pixels, and from the origin frame to the robot
        // one
        cv::Point2f tag_in_self =
          cs->getPosInSelf(storedObservations[i].first);
        tag_in_self.x =
          max(0, (int)(tag_in_self.x * scale_factor + width / 2));
        tag_in_self.y =
          max(0, (int)(-tag_in_self.y * scale_factor + height / 2));

        cv::circle(img, tag_in_self, 5, cv::Scalar(0, 0, 0), -1);
      }
    } else if (obsType == "compass") {
      for (unsigned int i = 0; i < storedObservations.size(); i++) {
        cv::Point2f opponent = storedObservations[i].first;
        cv::Point2f tmp;

        Angle goalDirInWorld(opponent.x);

        // trick store angle and quality
        Angle trunkYaw =
          cs->getTrunkYawInWorld(); // yaw of the trunk in the world

        Angle goalDirInSelf = goalDirInWorld - trunkYaw;
        // opponentGoal uses offset but has inverted sign with robot
        // orientation
        Angle oppGoalDirInSelf =
          -CompassObservation::compassToField(goalDirInSelf);

        double x = cos(oppGoalDirInSelf) * 10.0;
        double y = sin(oppGoalDirInSelf) * 10.0;

        tmp.x = max(0, (int)(x * scale_factor + width / 2));
        tmp.y = max(0, (int)(-y * scale_factor + height / 2));

        cv::Point2i center(width / 2, height / 2);
        cv::Scalar lineColor(0, 0, 255 * opponent.y); // quality is in y
        cv::line(img, center, tmp, lineColor, 1);
      }
    } else if (obsType == "clipping") {
      int idx = 0;
      while (idx < (int)storedObservations.size()) {
        int start_idx = idx;
        int line_nb = (int)storedObservations[idx].first.x;
        int has_corner = (int)storedObservations[idx].first.y;
        idx++;
        for (int k = 0; k < line_nb; k++) {
          if (idx >= (int)storedObservations.size() - 1) break;  // au cas où ...
          cv::Point2f A = cs->getPosInSelf(storedObservations[idx].first);
          cv::Point2f B = cs->getPosInSelf(storedObservations[idx + 1].first);
          A.x = max(0, (int)(A.x * scale_factor + width / 2));
          A.y = max(0, (int)(-A.y * scale_factor + height / 2));
          B.x = max(0, (int)(B.x * scale_factor + width / 2));
          B.y = max(0, (int)(-B.y * scale_factor + height / 2));
          cv::line(img, A, B, cv::Scalar::all(255), 2);
          idx += 2;
        }
        if (idx >= (int)storedObservations.size()) break;  // au cas où ...
        if (has_corner && idx == start_idx + line_nb * 2 + 1) {
          cv::Point2f C = cs->getPosInSelf(storedObservations[idx].first);
          C.x = max(0, (int)(C.x * scale_factor + width / 2));
          C.y = max(0, (int)(-C.y * scale_factor + height / 2));
          cv::circle(img, C, 5, cv::Scalar(255, 255, 255), CV_FILLED);
          idx++;
        }
      }
    }
  }
  globalMutex.unlock();

  return img;
}

  std::vector<cv::Point2f> Robocup::keepFrontRobots(std::vector<cv::Point2f> & robots) {
    //TODO, this function should take into account the size of a standard robot and hide any candidate that should be behind another robot
    return robots;
  }
  
void Robocup::run() { launch(); }

void Robocup::closeCamera() {
  if (pipeline.isFilterPresent("source")) {
    std::cerr << "Someone asked to close camera in Robocup, not implemented "
                 "for PtGrey"
              << std::endl;
  } else {
    std::cout << "source filter not found (camera might not be closed properly)"
              << std::endl;
  }
}

TimeStamp Robocup::getNowTS() const{
  if (isFakeMode()) {
    return sourceTS;
  }
  return TimeStamp::now();
}

bool Robocup::isFakeMode() const{
  return _scheduler->getServices()->model->isFakeMode();
}

void Robocup::ballClear() { 
    ballStackFilter->clear(); 
}

void Robocup::robotsClear() { 
    robotFilter->clear();
}

void Robocup::ballReset(float x, float y) {
  ballStackFilter->reset(x, y);
}

void Robocup::setLogMode(const std::string path) {
  _scheduler->getServices()->model->loadReplays(path);

  std::cout << "Loaded replay" << std::endl;
}

void Robocup::startLoggingLowLevel(const std::string & path) {
  std::cout << DEBUG_INFO << ": " <<  path  << std::endl;
  _scheduler->getServices()->model->startNamedLog(path);
}

void Robocup::stopLoggingLowLevel(const std::string & path) {
  out.log("Saving lowlevel log to: %s", path.c_str());
  TimeStamp start_save = TimeStamp::now();
  _scheduler->getServices()->model->stopNamedLog(path);
  TimeStamp end_save = TimeStamp::now();
  out.log("Lowlevel logs saved in %f seconds", diffSec(start_save, end_save));
}

int Robocup::getFrames() { return pipeline.frames; }

double Robocup::getLastUpdate() const {
  return diffMs(lastTS, getNowTS());
}

}
