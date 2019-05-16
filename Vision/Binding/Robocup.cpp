#include "Binding/Robocup.hpp"
#include <iostream>
#include <unistd.h>

#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "Localisation/Ball/BallStackFilter.hpp"
#include "Localisation/Ball/SpeedEstimator.hpp"

#include "Localisation/Robot/RobotFilter.hpp"

#include "scheduler/MoveScheduler.h"

#include "Filters/Features/FeaturesProvider.hpp"
#include "Filters/Features/TagsDetector.hpp"

#include "CameraState/CameraState.hpp"
#include "Utils/Drawing.hpp"
#include "Utils/Interface.h"
#include "Utils/PtGreyExceptions.hpp"

#include "rhoban_geometry/point.h"

#include "rhoban_utils/timing/benchmark.h"

#include "RhIO.hpp"
#include "robocup_referee/constants.h"

#include "moves/Head.h"
#include <rhoban_utils/logging/logger.h>
#include <rhoban_utils/util.h>

#include "services/DecisionService.h"
#include "services/LocalisationService.h"
#include "services/ModelService.h"
#include "services/RefereeService.h"
#include "services/ViveService.h"
#include <cmath>
#include <cstdlib>
#include <sstream>
#include <string>
#include <unistd.h>

#include <vector>

#include <utility>
#include <algorithm>

static rhoban_utils::Logger out("vision_robocup");

using namespace hl_monitoring;
using namespace Vision::Localisation;
using namespace rhoban_utils;
using namespace rhoban_geometry;

using namespace std;
using namespace std::chrono;

using namespace Vision::Utils;

using robocup_referee::Constants;
using Vision::Filters::TagsDetector;
using Vision::Utils::CameraState;
using Vision::Utils::ImageLogger;

namespace Vision
{
Robocup::Robocup(MoveScheduler* scheduler)
  : Application()
  , imageDelay(0)
  , manual_logger("manual_logs", true, 10000)
  , moving_ball_logger("moving_ball_logs", false, 1000)
  , autologMovingBall(false)
  , game_logger("game_logs", false, 15 * 60 * 40)
  ,  // Allows 15 minutes at 40 fps ->
  autolog_games(false)
  , logBallExtraTime(2.0)
  , writeBallStatus(false)
  , _scheduler(scheduler)
  , benchmark(false)
  , benchmarkDetail(0)
  , cs(new CameraState(scheduler))
  ,  // TODO: maybe put the name of the param file elsewhere
  activeSource(false)
  , clearRememberObservations(false)
  , detectedFeatures(new Field::POICollection())
  , detectedBalls(new std::vector<cv::Point3f>())
  , detectedRobots(new std::vector<cv::Point3f>())
  , wasHandled(false)
  , wasFallen(false)
  , ignoreOutOfFieldBalls(true)
{
  ballStackFilter = new BallStackFilter(cs);
  robotFilter = new RobotFilter(cs);
  ballSpeedEstimator = new SpeedEstimator();
  initObservationTypes();

  for (std::string obs : observationTypes)
  {
    rememberObservations[obs] = std::vector<std::pair<cv::Point2f, float>>();
  }

  out.log("Starting Robocup Pipeline");
  pipeline.setCameraState(cs);
  initImageHandlers();
  loadFile();
  _doRun = true;
  Filter::GPU_ON = gpuOn;
  if (pathToLog != "")
  {
    // The low level info will come from a log
    setLogMode(pathToLog);
  }
  if (viveLogPath != "")
  {
    setViveLog(viveLogPath);
  }
  scheduler->getServices()->localisation->setRobocup(this);
  _runThread = new std::thread(std::bind(&Robocup::run, this));
}

Robocup::Robocup(const std::string& configFile, MoveScheduler* scheduler)
  : imageDelay(0)
  , manual_logger("manual_logs", true, 10000)
  , moving_ball_logger("moving_ball_logs", false, 1000)
  , autologMovingBall(false)
  , game_logger("game_logs", true, 15 * 60 * 40)
  ,  // Allows 15 minutes at 40 fps
  autolog_games(false)
  , logBallExtraTime(2.0)
  , writeBallStatus(false)
  , benchmark(false)
  , benchmarkDetail(0)
  , _runThread(NULL)
  , cs(new CameraState(scheduler))
  , activeSource(false)
  , clearRememberObservations(false)
  , detectedFeatures(new Field::POICollection())
  , detectedBalls(new std::vector<cv::Point3f>())
  , detectedRobots(new std::vector<cv::Point3f>())
  , wasHandled(false)
  , wasFallen(false)
  , ignoreOutOfFieldBalls(true)
{
  ballStackFilter = new BallStackFilter(cs);
  robotFilter = new RobotFilter(cs);
  ballSpeedEstimator = new SpeedEstimator();
  initObservationTypes();

  for (std::string obs : observationTypes)
  {
    rememberObservations[obs] = std::vector<std::pair<cv::Point2f, float>>();
  }
  pipeline.setCameraState(cs);
  _scheduler = scheduler;
  initImageHandlers();
  loadFile(configFile);
  _doRun = true;
  Filter::GPU_ON = gpuOn;
  if (pathToLog != "")
  {
    // The low level info will come from a log
    setLogMode(pathToLog);
  }
  if (viveLogPath != "")
  {
    setViveLog(viveLogPath);
  }
  scheduler->getServices()->localisation->setRobocup(this);
}

Robocup::~Robocup()
{
  delete ballStackFilter;
  _doRun = false;
  if (_runThread != NULL)
  {
    _runThread->join();
    delete _runThread;
  }
}

void Robocup::startLogging(unsigned int timeMS, const std::string& logDir)
{
  // If logDir is empty a name session is generated automatically in manual_logger
  logMutex.lock();
  manual_logger.initSession(logDir);
  startLoggingLowLevel(manual_logger.getSessionPath() + "/lowLevel.log");
  endLog = TimeStamp(getNowTS() + milliseconds(timeMS));
  logMutex.unlock();
}

void Robocup::endLogging()
{
  logMutex.lock();
  // Telling the low level to stop logging and to dump the info
  stopLoggingLowLevel(manual_logger.getSessionPath() + "/lowLevel.log");
  manual_logger.endSession();
  // TODO: examine if logMutex can be closed earlier
  logMutex.unlock();
}

void Robocup::applyKick(double x, double y)
{
  // ballStackFilter->applyKick(x / 100.0, y / 100.0);
}

cv::Mat Robocup::getImg(const std::string& name, int wishedWidth, int wishedHeight, bool gray)
{
  cv::Mat original, scaled, final;
  if (name == "Tagged")
  {
    original = getTaggedImg();
  }
  else
  {
    try
    {
      original = pipeline.get(name).getImg()->clone();
    }
    catch (const std::out_of_range& o)
    {
      throw std::runtime_error("Image not found : '" + name + "'");
    }
  }
  cv::resize(original, scaled, cv::Size(wishedWidth, wishedHeight));
  if (gray)
  {
    cv::cvtColor(scaled, final, CV_RGB2GRAY);
  }
  else
  {
    final = scaled;
  }
  return final;
}

Json::Value Robocup::toJson() const
{
  // Writing stream
  Json::Value v = Application::toJson();
  v["viveLogPath"] = viveLogPath;
  v["benchmark"] = benchmark;
  v["benchmarkDetail"] = benchmarkDetail;
  v["imageDelay"] = imageDelay;
  v["autologMovingBall"] = autologMovingBall;
  v["autologGames"] = autolog_games;
  v["logBallExtraTime"] = logBallExtraTime;
  v["writeBallStatus"] = writeBallStatus;
  v["ignoreOutOfFieldBalls"] = ignoreOutOfFieldBalls;
  v["feature_providers"] = vector2Json(featureProviders);
  for (const SpecialImageHandler& sih : imageHandlers)
  {
    v[sih.name] = sih.display;
  }
  return v;
}

void Robocup::fromJson(const Json::Value& v, const std::string& dir_name)
{
  Application::fromJson(v, dir_name);
  rhoban_utils::tryRead(v, "viveLogPath", &viveLogPath);
  rhoban_utils::tryRead(v, "benchmark", &benchmark);
  rhoban_utils::tryRead(v, "benchmarkDetail", &benchmarkDetail);
  rhoban_utils::tryRead(v, "imageDelay", &imageDelay);
  rhoban_utils::tryRead(v, "autologMovingBall", &autologMovingBall);
  rhoban_utils::tryRead(v, "autologGames", &autolog_games);
  rhoban_utils::tryRead(v, "logBallExtraTime", &logBallExtraTime);
  rhoban_utils::tryRead(v, "writeBallStatus", &writeBallStatus);
  rhoban_utils::tryRead(v, "ignoreOutOfFieldBalls", &ignoreOutOfFieldBalls);
  rhoban_utils::tryReadVector<std::string>(v, "featureProviders", &featureProviders);
  for (SpecialImageHandler& sih : imageHandlers)
  {
    rhoban_utils::tryRead(v, sih.name, &sih.display);
  }
}

void Robocup::init()
{
  Application::init();
  lastTS = ::rhoban_utils::TimeStamp::fromMS(0);

  initRhIO();
}

void Robocup::initImageHandlers()
{
  imageHandlers.push_back(SpecialImageHandler(
      "TaggedImg", 640, 480, [this](int width, int height) { return this->getTaggedImg(width, height); }));
  imageHandlers.push_back(SpecialImageHandler(
      "RadarImg", 640, 480, [this](int width, int height) { return this->getRadarImg(width, height); }));
}

void Robocup::initRhIO()
{
  // If a command has already been created do not pass here again
  if (RhIO::Root.commandExist("Vision/logLocal"))
  {
    return;
  }
  RhIO::Root.newStr("/Vision/cameraStatus")->defaultValue("");
  RhIO::Root.newFloat("/Vision/lastUpdate")->defaultValue(-1)->comment("Time since last update [ms]");
  // Init interface with RhIO
  if (isFakeMode())
  {  /// Highgui is not available on robot
    RhIO::Root.newCommand("Vision/showFilters", "Display the given filters",
                          [this](const std::vector<std::string>& args) -> std::string {
                            if (args.size() < 1)
                            {
                              throw std::runtime_error("Usage: showFilters <name1> <name2> ...");
                            }
                            for (const std::string& name : args)
                            {
                              try
                              {
                                pipeline.get(name).display = true;
                              }
                              catch (const std::out_of_range& exc)
                              {
                                throw std::runtime_error("Filter " + name + " is not found in pipeline");
                              }
                            }
                            return "Filters are now displayed";
                          });
    RhIO::Root.newCommand("Vision/hideFilters", "Hide the given filters",
                          [this](const std::vector<std::string>& args) -> std::string {
                            if (args.size() < 1)
                            {
                              throw std::runtime_error("Usage: hideFilters <name1> <name2> ...");
                            }
                            for (const std::string& name : args)
                            {
                              try
                              {
                                pipeline.get(name).display = false;
                                cv::destroyWindow(name);
                              }
                              catch (const std::out_of_range& exc)
                              {
                                throw std::runtime_error("Filter " + name + " is not found in pipeline");
                              }
                            }
                            return "Filters are now hidden";
                          });
  }
  RhIO::Root.newCommand("Vision/logLocal",
                        "Starts logging for a specified duration. Images are "
                        "saved on board for now.",
                        [this](const std::vector<std::string>& args) -> std::string {
                          if (args.size() < 1)
                          {
                            throw std::runtime_error("Usage: logLocal <duration[s]> <opt:log_dir>");
                          }
                          double duration = std::stof(args[0]);
                          std::string logDir("");
                          if (args.size() >= 2)
                          {
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
  RhIO::Root.newBool("/Vision/benchmark")->defaultValue(benchmark)->comment("Is logging activated ?");
  RhIO::Root.newInt("/Vision/benchmarkDetail")->defaultValue(benchmarkDetail)->comment("Depth of print for benchmark");
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
  for (const SpecialImageHandler& sih : imageHandlers)
  {
    std::string prefix = "Vision/" + sih.name;
    RhIO::Root.newFloat(prefix + "_scale")->defaultValue(1.0)->comment("");
    RhIO::Root.newFrame(prefix, "", RhIO::FrameFormat::BGR);
  }

  ballStackFilter->bindToRhIO("ballStack", "ballStack");

  robotFilter->bindToRhIO("robotFilter", "robotFilter");
}

void Robocup::initObservationTypes()
{
  observationTypes = { "ball", "robot", "tag" };
  for (Field::POIType type : Field::getPOITypeValues())
  {
    observationTypes.push_back(Field::poiType2String(type));
  }
}

void Robocup::finish()
{
}

void Robocup::step()
{
  if (clearRememberObservations)
  {
    for (auto& entry : rememberObservations)
    {
      entry.second.clear();
    }
    clearRememberObservations = false;
  }

  // Sometimes vision is useless and even bug prone, in this case, cancel the
  // step
  DecisionService* decision = _scheduler->getServices()->decision;
  bool handled = decision->handled;
  bool fallen = decision->isFallen;
  if (embedded && (handled || fallen))
  {
    publishToRhIO();
    std::ostringstream oss;
    // Updating handled/fallen and sending message
    if (!wasHandled && handled)
    {
      out.log("Disabling vision (handled)");
      wasHandled = true;
    }
    if (wasHandled && !handled)
    {
      out.log("Robot is not handled anymore");
      wasHandled = false;
    }
    if (!wasFallen && fallen)
    {
      wasFallen = true;
      out.log("Disabling vision (fallen)");
    }
    if (wasFallen && !fallen)
    {
      out.log("Robot is not fallen anymore");
      wasFallen = false;
    }
    // Sleeping and waiting next step
    int ms_sleep = 10;
    usleep(ms_sleep * 1000);
    return;
  }
  if (wasHandled || wasFallen)
  {
    out.log("Starting to step vision again");
    wasHandled = false;
    wasFallen = false;
  }

  // If the camera is unplugged or doesn't respond for too long, we should avoid
  // keeping the filters data
  {
    double timeSinceLastFrame = diffSec(lastTS, getNowTS());
    if (timeSinceLastFrame > 5)
    {
      out.warning("no frame for %f, reseting the ball filter", timeSinceLastFrame);

      // Resetting the ball stack filter
      ballStackFilter->clear();

      // Telling the localisation
      _scheduler->getServices()->localisation->setNoBall();
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
  try
  {
    Application::step();
    activeSource = true;
    // If Vision application has finished, ask for scheduler to shut down
    if (!isActive())
    {
      out.log("Vision exiting, asking to scheduler to shut down");
      _scheduler->askQuit();
    }
  }
  catch (const PtGreyException& exc)
  {
    globalMutex.unlock();
    Benchmark::close("Pipeline");
    Benchmark::close("Vision + Localisation", benchmark, benchmarkDetail);
    activeSource = false;
    out.warning("Failed vision step: '%s'", exc.what());
    publishToRhIO();
    int sleep_time_ms = 100;
    usleep(sleep_time_ms * 1000);
    return;
  }
  catch (const PtGreyConnectionException& exc)
  {
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

  for (SpecialImageHandler& sih : imageHandlers)
  {
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
      RhIO::Root.framePush(prefix, img_width, img_height, sih.lastImg.data, sih.getSize());
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
  if (isFakeMode())
  {
    double ts = pipeline.getTimestamp().getTimeMS();
    _scheduler->getServices()->model->setReplayTimestamp(ts / 1000.0);
  }
}

void Robocup::importFromRhIO()
{
  autologMovingBall = RhIO::Root.getValueBool("/Vision/autologMovingBall").value;
  autolog_games = RhIO::Root.getValueBool("/Vision/autologGames").value;
  logBallExtraTime = RhIO::Root.getValueFloat("/Vision/logBallExtraTime").value;
  benchmark = RhIO::Root.getValueBool("/Vision/benchmark").value;
  benchmarkDetail = RhIO::Root.getValueInt("/Vision/benchmarkDetail").value;
  imageDelay = RhIO::Root.getValueInt("/Vision/imageDelay").value;
  angularPitchTolerance = RhIO::Root.getValueFloat("/Vision/angularPitchTolerance").value;
  // Import size update for images
  for (SpecialImageHandler& sih : imageHandlers)
  {
    std::string prefix = "Vision/" + sih.name;
    sih.scale = RhIO::Root.getValueFloat(prefix + "_scale").value;
  }
}

void Robocup::publishToRhIO()
{
  RhIO::Root.setFloat("/Vision/lastUpdate", diffMs(lastTS, getNowTS()));
  std::string cameraStatus = getCameraStatus();
  RhIO::Root.setStr("/Vision/cameraStatus", cameraStatus);
}

std::string Robocup::getCameraStatus() const
{
  if (!activeSource)
  {
    return "Connection lost";
  }
  DecisionService* decision = _scheduler->getServices()->decision;
  if (decision->handled || decision->isFallen)
  {
    return "Inactive (handled or fallen)";
  }
  return "Active";
}

void Robocup::readPipeline()
{
  featuresMutex.lock();
  // Ball and robots are cleared after every step (used internally)
  detectedBalls->clear();
  detectedRobots->clear();
  for (const auto& provider_name : featureProviders)
  {
    try
    {
      const Filters::FeaturesProvider& provider =
          dynamic_cast<const Filters::FeaturesProvider&>(pipeline.get(provider_name));
      // Balls import
      std::vector<cv::Point2f> balls_in_img = provider.getBalls();
      for (const cv::Point2f& ball_in_img : balls_in_img)
      {
        cv::Point2f world_pos = cs->worldPosFromImg(ball_in_img.x, ball_in_img.y);
        detectedBalls->push_back(cv::Point3f(world_pos.x, world_pos.y, Constants::field.ball_radius));
      }
      // POI update
      std::map<Field::POIType, std::vector<cv::Point2f>> pois_in_img = provider.getPOIs();
      for (const auto& entry : pois_in_img)
      {
        Field::POIType poi_type = entry.first;
        for (const cv::Point2f& feature_pos_in_img : entry.second)
        {
          cv::Point2f world_pos = cs->worldPosFromImg(feature_pos_in_img.x, feature_pos_in_img.y);
          detectedFeatures->operator[](poi_type).push_back(cv::Point3f(world_pos.x, world_pos.y, 0));
        }
      }
      // Robot import
      std::vector<cv::Point2f> robots_in_img = provider.getRobots();  // TODO: add robot info (color)
      for (const cv::Point2f& robot_in_img : robots_in_img)
      {
        cv::Point2f world_pos = cs->worldPosFromImg(robot_in_img.x, robot_in_img.y);
        detectedRobots->push_back(cv::Point3f(world_pos.x, world_pos.y, Constants::field.ball_radius));
      }
    }
    catch (const std::bad_cast& e)
    {
      out.error("%s: Failed to import features, check pipeline. Exception: %s", DEBUG_INFO.c_str(), e.what());
    }
    catch (const std::runtime_error& exc)
    {
      out.error("%s: Failed to import features, runtime_error. Exception: %s", DEBUG_INFO.c_str(), exc.what());
    }
  }
  featuresMutex.unlock();

  // Tags (temporarily disabled, to reactivate, require to add a 'tagProvider' similar to featureProviders
  //  for (const std::string& tagProviderName : tagProviders)
  //  {
  //    Vision::Filter& tagFilter = pipeline.get(tagProviderName);
  //    cv::Size size = pipeline.get(tagProviderName).getImg()->size();
  //    tagsMutex.lock();
  //    detectedTimestamp = pipeline.getTimestamp().getTimeMS() / 1000.0;
  //    try
  //    {
  //      const TagsDetector& tagProvider = dynamic_cast<const TagsDetector&>(tagFilter);
  //      const std::vector<TagsDetector::Marker>& new_tags = tagProvider.getDetectedMarkers();
  //      for (const TagsDetector::Marker& marker : new_tags)
  //      {
  //        Eigen::Vector3d pos_camera;
  //        cv::cv2eigen(marker.tvec, pos_camera);
  //        Eigen::Vector3d marker_pos_in_world = cs->getWorldPosFromCamera(pos_camera);
  //
  //        // Adding Marker to detectedTagsb
  //        detectedTagsIndices.push_back(marker.id);
  //        detectedTagsPositions.push_back(marker_pos_in_world);
  //
  //        // Calculating the center of the tags on the image (x, y)
  //        // TODO, if the barycenter is not good enough, do better (crossing the
  //        // diags?)
  //        Eigen::Vector2d avg_in_img(0, 0);
  //        for (unsigned int i = 0; i < marker.corners.size(); i++)
  //        {
  //          avg_in_img += Eigen::Vector2d(marker.corners[i].x, marker.corners[i].y);
  //        }
  //        avg_in_img /= 4.0;
  //        // Rescaling to cameraModel image
  //        avg_in_img(0) *= cs->getCameraModel().getImgWidth() / size.width;
  //        avg_in_img(1) *= cs->getCameraModel().getImgHeight() / size.height;
  //        // Undistort position
  //        cv::Point2f avg_in_corrected;
  //        avg_in_corrected = cs->getCameraModel().toCorrectedImg(eigen2CV(avg_in_img));
  //        // Using a pair instead than a cv::Point so the structure is usable even
  //        // without opencv (will be read by the low level)
  //        std::pair<float, float> pair_in_img(avg_in_img(0), avg_in_img(1));
  //        std::pair<float, float> pair_undistorded(avg_in_corrected.x, avg_in_corrected.y);
  //        detectedTagsCenters.push_back(pair_in_img);
  //        detectedTagsCentersUndistort.push_back(pair_undistorded);
  //      }
  //    }
  //    catch (const std::bad_cast& exc)
  //    {
  //      tagsMutex.unlock();
  //      throw std::runtime_error("Invalid type for filter 'TagsDetector'");
  //    }
  //    catch (...)
  //    {
  //      tagsMutex.unlock();
  //      throw;
  //    }
  //    tagsMutex.unlock();
  //  }
}

void Robocup::getUpdatedCameraStateFromPipeline()
{
  csMutex.lock();

  // Backup values
  lastTS = sourceTS;

  sourceTS = cs->getTimeStamp();

  // TODO: identify if this part is only debug
  if (!isFakeMode())
  {
    double timeSinceLastFrame = diffSec(lastTS, sourceTS);
    if (timeSinceLastFrame > 2 || timeSinceLastFrame < 0)
    {
      out.warning("Suspicious elapsed time: %f [s]", timeSinceLastFrame);
    }
  }

  // EXPERIMENTAL:
  //
  // modification linked to the possibility that 'Source' filter provides the
  // cameraState (from SourceVideoProtobuf)
  cs = pipeline.getCameraState();
  ballStackFilter->updateCS(cs);
  robotFilter->updateCS(cs);

  csMutex.unlock();
}

void Robocup::loggingStep()
{
  TimeStamp now = getNowTS();

  DecisionService* decision = _scheduler->getServices()->decision;
  RefereeService* referee = _scheduler->getServices()->referee;

  // Capture src_filter and throws a std::runtime_error if required
  const Filter& src_filter = pipeline.get("source");
  ImageLogger::Entry entry(pipeline.getTimestamp(), *(src_filter.getImg()));

  logMutex.lock();
  // Handling manual logs
  bool dumpManualLogs = false;
  if (manual_logger.isActive())
  {
    if (endLog < now)
    {  // If time is elapsed: close log
      dumpManualLogs = true;
    }
    else
    {  // If there is still time left, add entry
      try
      {
        manual_logger.pushEntry(entry);
      }
      catch (const ImageLogger::SizeLimitException& exc)
      {
        out.warning("Automatically stopping manual log because size limit was reached");
        dumpManualLogs = true;
      }
    }
  }
  // Handling moving ball logs
  if (decision->isBallMoving)
  {
    lastBallMoving = now;
  }
  double elapsedSinceBallMoving = diffSec(lastBallMoving, now);
  // Status of autoLog
  bool autoLogActive = moving_ball_logger.isActive();
  bool startAutoLog = autologMovingBall && !autoLogActive && decision->isMateKicking;
  bool stopAutoLog = autoLogActive && !(elapsedSinceBallMoving < logBallExtraTime || decision->isMateKicking);
  bool useAutoLogEntry = startAutoLog || (autoLogActive && !stopAutoLog);
  // Starting autoLog
  if (startAutoLog)
  {
    moving_ball_logger.initSession();
    out.log("Starting a session at '%s'", moving_ball_logger.getSessionPath().c_str());
    std::string lowLevelPath = moving_ball_logger.getSessionPath() + "/lowLevel.log";
    startLoggingLowLevel(lowLevelPath);
  }
  // Trying to log entry (can fail is maxSize is reached)
  if (useAutoLogEntry)
  {
    try
    {
      moving_ball_logger.pushEntry(entry);
    }
    catch (const ImageLogger::SizeLimitException& exc)
    {
      stopAutoLog = true;
    }
  }

  // Status of game_logs
  bool is_playing = referee->isPlaying();
  bool gameLogActive = game_logger.isActive();
  bool useGameLogEntry = autolog_games && is_playing;
  bool startGameLog = !gameLogActive && useGameLogEntry;
  bool stopGameLog = gameLogActive && !useGameLogEntry;
  if (startGameLog)
  {
    game_logger.initSession();
    out.log("Starting a session at '%s'", game_logger.getSessionPath().c_str());
    std::string lowLevelPath = game_logger.getSessionPath() + "/lowLevel.log";
    startLoggingLowLevel(lowLevelPath);
  }
  if (useGameLogEntry)
  {
    try
    {
      game_logger.pushEntry(entry);
    }
    catch (const ImageLogger::SizeLimitException& exc)
    {
      stopGameLog = true;
    }
  }
  logMutex.unlock();

  // Writing logs is delayed until logMutex has been unlocked to avoid
  // unnecessary lock of ressources
  if (dumpManualLogs)
  {
    _scheduler->stopMove("head", 0.5);
    endLogging();
  }
  if (stopAutoLog)
  {
    std::string lowLevelPath = moving_ball_logger.getSessionPath() + "/lowLevel.log";
    stopLoggingLowLevel(lowLevelPath);
    moving_ball_logger.endSession();
  }
  if (stopGameLog)
  {
    std::string lowLevelPath = game_logger.getSessionPath() + "/lowLevel.log";
    stopLoggingLowLevel(lowLevelPath);
    game_logger.endSession();
  }
}

void Robocup::updateBallInformations()
{
  std::vector<Eigen::Vector3d> positions;
  // Getting candidates in ball by ROI
  for (const cv::Point3f& ball_pos_in_world : *detectedBalls)
  {
    try
    {
      Eigen::Vector3d ballInWorld = cv2Eigen(ball_pos_in_world);
      if (ignoreOutOfFieldBalls && cs->has_camera_field_transform)
      {
        Eigen::Vector3d ballInField = cs->field_from_camera * cs->worldToCamera * ballInWorld;
        // OPTION: Margin could be added here
        if (!Constants::field.isInArena(cv::Point2f(ballInField.x(), ballInField.y())))
        {
          out.warning("Ignoring a ball candidate outside of the field at (%f,%f)", ballInField.x(), ballInField.y());
          continue;
        }
      }
      // Usual code
      positions.push_back(ballInWorld);
      ballSpeedEstimator->update(cs->getTimeStamp(), Eigen::Vector2d(ballInWorld(0), ballInWorld(1)));
    }
    catch (const std::runtime_error& exc)
    {
      out.warning("Ignoring a candidate at (%f,%f) because of '%s'", ball_pos_in_world.x, ball_pos_in_world.y,
                  exc.what());
    }
  }
  // Positions are transmitted in the world referential
  ballStackFilter->newFrame(positions);

  // Broadcast information to localisation service
  // Sending data to the loc
  LocalisationService* loc = _scheduler->getServices()->localisation;

  if (ballStackFilter->getCandidates().size() > 0)
  {
    auto bestCandidate = ballStackFilter->getBest();
    double bsfMaxScore = ballStackFilter->getMaximumScore();
    Point ballSpeed = ballSpeedEstimator->getUsableSpeed();
    loc->setBallWorld(bestCandidate.object, bestCandidate.score / bsfMaxScore, ballSpeed, cs->getTimeStamp());
  }
  else
  {
    loc->setNoBall();
  }

  /// If active: write ballStatus
  if (writeBallStatus)
  {
    // Some properties are shared for the frame
    double time = getNowTS().getTimeSec();
    for (const Eigen::Vector3d& pos_in_world : positions)
    {
      Point ball_pos_in_field = loc->worldToField(pos_in_world);
      Point robot_pos = loc->getFieldPos();
      double field_dir = normalizeRad(loc->getFieldOrientation());
      // Entry format:
      // TimeStamp, ballX, ballY, robotX, robotY, fieldDir
      out.log("ballStatusEntry: %lf,%f,%f,%f,%f,%f", time, ball_pos_in_field.x, ball_pos_in_field.y, robot_pos.x,
              robot_pos.y, field_dir);
    }
  }
}

std::unique_ptr<Field::POICollection> Robocup::stealFeatures()
{
  std::lock_guard<std::mutex> lock(featuresMutex);
  std::unique_ptr<Field::POICollection> tmp = std::move(detectedFeatures);
  detectedFeatures.reset(new Field::POICollection());
  return std::move(tmp);
}

void Robocup::stealTags(std::vector<int>& indices, std::vector<Eigen::Vector3d>& positions,
                        std::vector<std::pair<float, float>>& centers,
                        std::vector<std::pair<float, float>>& undistorded_centers, double* timestamp)
{
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

cv::Mat Robocup::getTaggedImg()
{
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
cv::Mat Robocup::getTaggedImg(int width, int height)
{
  if (cs == NULL)
    throw std::runtime_error("TaggedImg not ready");
  cv::Mat tmp, img, tmp_small;
  globalMutex.lock();

  tmp = pipeline.get("source").getImg()->clone();

  cv::Rect2f img_rect(cv::Point2f(), cv::Point2f(width, height));

  cv::resize(tmp, tmp_small, cv::Size(width, height));

  cv::cvtColor(tmp_small, img, CV_YCrCb2BGR);

  {
    // Drawing candidates of balls
    auto candidates = ballStackFilter->getCandidates();
    int k = 0;
    for (auto& candidate : candidates)
    {
      k++;
      Eigen::Vector3d cpos = candidate.object;
      try
      {
        cv::Point2f pos = cs->imgXYFromWorldPosition(cpos);

        // I candidate is outside of the image, ignore it
        if (!img_rect.contains(pos))
          continue;

        // Draw ball candidate
        cv::circle(img, pos, 6 * candidate.score, cv::Scalar(255, 255, 0), CV_FILLED);

        // Write candidate number
        std::stringstream ss;
        ss << (candidates.size() - k);
        cv::putText(img, ss.str(), pos, cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(0, 0, 0), 1);

        // # Futur position of ball
        // Elapsed time
        double tag_ball_anticipation = 0.2;  // [s] (TODO: set as rhio parameters?)
        double elapsed = tag_ball_anticipation;
        elapsed += diffSec(cs->getTimeStamp(), getNowTS());
        // Compute futur position
        Point ball_usable_speed = ballSpeedEstimator->getUsableSpeed();
        Eigen::Vector3d ball_speed(ball_usable_speed.x, ball_usable_speed.y, 0);

        Eigen::Vector3d next_cpos = cpos + ball_speed * elapsed;
        cv::Point2f futur_pos = cs->imgXYFromWorldPosition(next_cpos);
        cv::line(img, pos, futur_pos, cv::Scalar(255, 255, 0), 2);
      }
      catch (const std::runtime_error& exc)
      {
      }
    }
  }

  {
    // Drawing candidates of robots
    auto candidates = robotFilter->getCandidates();
    int k = 0;
    for (auto& candidate : candidates)
    {
      k++;
      Eigen::Vector3d cpos = candidate.object;
      try
      {
        cv::Point2f pos = cs->imgXYFromWorldPosition(cv::Point2f(cpos.x(), cpos.y()));

        // If candidate is outside of the image, ignore it
        if (!img_rect.contains(pos))
          continue;

        // Draw robot candidate
        cv::circle(img, pos, 6 * candidate.score, cv::Scalar(255, 0, 255), CV_FILLED);

        // Write candidate number
        std::stringstream ss;
        ss << (candidates.size() - k);
        cv::putText(img, ss.str(), pos, cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(0, 0, 0), 1);
      }
      catch (const std::runtime_error& exc)
      {
      }
    }
  }

  // Tagging balls seen on current image
  for (const cv::Point3f& ballPosInWorld : *detectedBalls)
  {
    try
    {
      cv::Point center = cs->imgXYFromWorldPosition(cv2Eigen(ballPosInWorld));
      double ballRadius = cs->computeBallRadiusFromPixel(center);
      if (ballRadius > 0)
      {
        cv::circle(img, center, (int)ballRadius, cv::Scalar(0, 0, 0), 2);
      }
    }
    catch (const std::runtime_error& exc)
    {
      logger.warning("%s, cannot find imgXY from world for ballPos: '%s'", DEBUG_INFO, exc.what().c_str());
    }
  }

  // Drawing opponent robots
  auto candidates = ballStackFilter->getCandidates();

  double angleStep = M_PI / 10;
  std::vector<cv::Point2f> horizonKeypoints;
  Eigen::Vector3d cameraPos = cs->getWorldPosFromCamera(Eigen::Vector3d::Zero());
  Eigen::Vector3d cameraDir = cs->getWorldPosFromCamera(Eigen::Vector3d::UnitZ()) - cameraPos;
  for (double yaw = -M_PI; yaw <= M_PI; yaw += angleStep)
  {
    Eigen::Vector3d offset(cos(yaw), sin(yaw), 0);
    // skip to next value if object is behind camera plane
    if (cameraDir.dot(offset) <= 0)
      continue;
    Eigen::Vector3d target = cameraPos + offset;
    try
    {
      cv::Point2f p = cs->imgXYFromWorldPosition(target);
      if (img_rect.contains(p))
      {
        horizonKeypoints.push_back(p);
      }
    }
    catch (const std::runtime_error& exc)
    {
    }
  }
  for (size_t idx = 1; idx < horizonKeypoints.size(); idx++)
  {
    cv::line(img, horizonKeypoints[idx - 1], horizonKeypoints[idx], cv::Scalar(255, 0, 0), 2);
  }

  // TODO remove it and do something cleaner lates
  if (cs->has_camera_field_transform)
  {
    // Drawing field_lines
    cv::Mat camera_matrix, distortion_coeffs, rvec, tvec;
    camera_matrix = cs->getCameraModel().getCameraMatrix();
    distortion_coeffs = cs->getCameraModel().getDistortionCoeffs();
    affineToCV(cs->camera_from_field, &rvec, &tvec);

    cv::Scalar line_color(0, 0, 0);
    double line_thickness = 2.0;  // px
    int nb_segments = 10;
    Constants::field.tagLines(camera_matrix, distortion_coeffs, rvec, tvec, &img, line_color, line_thickness,
                              nb_segments);
    // Drawing tagged points
    ViveService* vive = _scheduler->getServices()->vive;
    if (vive->isActive())
    {
      Constants::field.tagPointsOfInterest(camera_matrix, distortion_coeffs, rvec, tvec, &img);
    }
    else
    {
      out.log("Vive is not active");
    }
  }

  globalMutex.unlock();

  return img;
}

cv::Mat Robocup::getRadarImg(int width, int height)
{
  if (cs == NULL)
    throw std::runtime_error("RadarImg not ready");
  cv::Mat img;
  img = cv::Mat(height, width, CV_8UC3, cv::Scalar(0, 30, 0));
  // Drawing the robot
  cv::circle(img, cv::Point2i(width / 2, height / 2), 5, cv::Scalar(0, 200, 200), -1);
  globalMutex.lock();

  std::vector<cv::Point2f> freshObservations;
  std::vector<int> delete_me;
  // scale_factor -> conversion [m] -> [px]
  float scale_factor = width / (2 * Constants::field.field_length);
  cv::Scalar ball_color = cv::Scalar(0, 0, 200);
  float discount = 0.05;

  // Drawing static distance marquers each meter (light circles are 0.5 meters)
  for (int i = 1; i < (1 + 2 * Constants::field.field_length); i++)
  {
    cv::circle(img, cv::Point2i(width / 2, height / 2), (i / 2.0) * scale_factor, cv::Scalar(0, 150, 0), 2 - (i % 2));
  }
  // Drawing vision cone
  rhoban_utils::Angle yaw = cs->getYaw();
  rhoban_utils::Angle half_aperture = cs->getCameraModel().getFOVY();
  cv::Point2i p1(width / 2 - height * sin(yaw + half_aperture), height / 2 - height * cos(yaw + half_aperture));
  cv::Point2i p2(width / 2 - height * sin(yaw - half_aperture), height / 2 - height * cos(yaw - half_aperture));

  cv::line(img, cv::Point2i(width / 2, height / 2), p1, cv::Scalar(0, 100, 100), 2);
  cv::line(img, cv::Point2i(width / 2, height / 2), p2, cv::Scalar(0, 100, 100), 2);
  // 0° orientation (front)
  cv::line(img, cv::Point2i(width / 2, height / 2), cv::Point2f(width / 2, 0), cv::Scalar(255, 0, 0), 1);

  for (std::string obsType : observationTypes)
  {
    std::vector<std::pair<cv::Point2f, float>>& storedObservations = rememberObservations[obsType];
    // Discounting and Killing observations
    delete_me.clear();
    freshObservations.clear();
    for (unsigned int i = 0; i < storedObservations.size(); i++)
    {
      auto scored_obs = storedObservations[i];
      // Discounting all the old observations
      storedObservations[i].second = storedObservations[i].second - discount;
      if (scored_obs.second < 0)
      {
        // This observation is considered dead now (checking this only once)
        delete_me.push_back(i);
      }
    }

    for (int i = delete_me.size() - 1; i > -1; i--)
    {
      // Erasing from the end of the vector to the start, so the smaller indexes
      // don't change.
      storedObservations.erase(storedObservations.begin() + delete_me[i]);
    }

    // Reading the fresh observations (observation type dependent)
    if (obsType == "ball")
    {
      for (const cv::Point3f& ballPosInWorld : *detectedBalls)
      {
        freshObservations.push_back(cv::Point2f(ballPosInWorld.x, ballPosInWorld.y));
      }
    }
    else if (obsType == "tag")
    {
      // Reading tags
      // Going from and image position to a position on the field, in the origin
      // (of world) frame
      for (unsigned int index = 0; index < detectedTagsCenters.size(); index++)
      {
        try
        {
          std::cout << "DetectedTagsCenter at " << detectedTagsCenters[index].first << ", "
                    << detectedTagsCenters[index].second << std::endl;
          // Going from and image position to a position on the field, in the
          // origin (of world) frame
          auto point = cs->worldPosFromImg(detectedTagsCenters[index].first, detectedTagsCenters[index].second);
          freshObservations.push_back(point);
        }
        catch (const std::runtime_error& exc)
        {
          // Ignore the candidate
        }
      }
    }
    else if (obsType == "robot")
    {
      for (const cv::Point3f& robotPosInWorld : *detectedRobots)
      {
        freshObservations.push_back(cv::Point2f(robotPosInWorld.x, robotPosInWorld.y));
      }
    }
    else
    {
      Field::POIType poiType = Field::string2POIType(obsType);
      for (const cv::Point3f& featurePosInWorld : detectedFeatures->operator[](poiType))
      {
        freshObservations.push_back(cv::Point2f(featurePosInWorld.x, featurePosInWorld.y));
      }
    }

    // Adding the observations to the old ones if need be, and updating the intensities
    for (const cv::Point2f& new_obs : freshObservations)
    {
      storedObservations.push_back(std::pair<cv::Point2f, float>(new_obs, 1.0));
    }

    // Drawing
    for (unsigned int i = 0; i < storedObservations.size(); i++)
    {
      // Going from meters to pixels, and from the origin frame to the robot one
      // TODO: question, why do we use the max here???
      cv::Point2f obs_in_self = cs->getPosInSelf(storedObservations[i].first);
      cv::Point2f obs_in_img(width / 2 - obs_in_self.y * scale_factor, height / 2 - obs_in_self.x * scale_factor);
      double default_radius = 3;  // [px]
      double marker_size = 8;
      double marker_thickness = 2;
      if (obsType == "robot")
      {
        cv::circle(img, obs_in_img, 15, cv::Scalar(200, 0, 200), -1);
      }
      else if (obsType == "ball")
      {
        cv::circle(img, obs_in_img, default_radius, ball_color, -1);
      }
      else if (obsType == "tag")
      {
        cv::circle(img, obs_in_img, 5, cv::Scalar(0, 0, 0), -1);
      }
      else
      {
        Field::POIType poiType = Field::string2POIType(obsType);
        switch (poiType)
        {
          case Field::POIType::PostBase:
            cv::circle(img, obs_in_img, default_radius, cv::Scalar(255, 255, 255), -1);
            break;
          case Field::POIType::X:
            cv::drawMarker(img, obs_in_img, cv::Scalar(255, 255, 255), cv::MarkerTypes::MARKER_TILTED_CROSS,
                           marker_size, marker_thickness);
            break;
          case Field::POIType::T:
            cv::drawMarker(img, obs_in_img, cv::Scalar(255, 255, 255), cv::MarkerTypes::MARKER_TRIANGLE_UP, marker_size,
                           marker_thickness);
            break;
          default:
            out.warning("Draw of POI of type '%s' is not implemented", obsType.c_str());
        }
      }
    }
  }
  globalMutex.unlock();

  return img;
}

std::vector<cv::Point2f> Robocup::keepFrontRobots(std::vector<cv::Point2f>& robots)
{
  // TODO, this function should take into account the size of a standard robot and hide any candidate that should be
  // behind another robot
  return robots;
}

void Robocup::run()
{
  launch();
}

void Robocup::closeCamera()
{
  if (pipeline.isFilterPresent("source"))
  {
    std::cerr << "Someone asked to close camera in Robocup, not implemented "
                 "for PtGrey"
              << std::endl;
  }
  else
  {
    std::cout << "source filter not found (camera might not be closed properly)" << std::endl;
  }
}

TimeStamp Robocup::getNowTS() const
{
  if (isFakeMode())
  {
    return sourceTS;
  }
  return TimeStamp::now();
}

bool Robocup::isFakeMode() const
{
  return _scheduler->getServices()->model->isFakeMode();
}

void Robocup::ballClear()
{
  ballStackFilter->clear();
}

void Robocup::robotsClear()
{
  robotFilter->clear();
}

void Robocup::ballReset(float x, float y)
{
  ballStackFilter->reset(x, y);
}

void Robocup::setLogMode(const std::string& path)
{
  _scheduler->getServices()->model->loadReplays(path);

  std::cout << "Loaded replay" << std::endl;
}

void Robocup::setViveLog(const std::string& path)
{
  _scheduler->getServices()->vive->loadLog(path);
}

void Robocup::startLoggingLowLevel(const std::string& path)
{
  std::cout << DEBUG_INFO << ": " << path << std::endl;
  _scheduler->getServices()->model->startNamedLog(path);
}

void Robocup::stopLoggingLowLevel(const std::string& path)
{
  out.log("Saving lowlevel log to: %s", path.c_str());
  TimeStamp start_save = TimeStamp::now();
  _scheduler->getServices()->model->stopNamedLog(path);
  TimeStamp end_save = TimeStamp::now();
  out.log("Lowlevel logs saved in %f seconds", diffSec(start_save, end_save));
}

int Robocup::getFrames()
{
  return pipeline.frames;
}

double Robocup::getLastUpdate() const
{
  return diffMs(lastTS, getNowTS());
}

}  // namespace Vision
