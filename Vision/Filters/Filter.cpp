#include <sstream>
#include <stdexcept>
#include <chrono>
#include <string>
#include <exception>
#include <locale>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "Filters/Filter.hpp"
#include "Filters/Pipeline.hpp"
#include <set>
#include <map>
#include "CameraState/CameraState.hpp"
#include "rhoban_utils/timing/benchmark.h"
#include "rhoban_utils/util.h"

#include "Utils/OpencvUtils.h"

#include "RhIO.hpp"

#include "rhoban_utils/logging/logger.h"

using namespace rhoban_geometry;
using namespace rhoban_utils;

static rhoban_utils::Logger out("Filter");

namespace Vision {

bool Filter::GPU_ON = false;

std::vector<std::string> Filter::listOfPresentWindows;

Filter::Filter(const std::string &n, const Dependencies &dependencies)
    : display(false),
      _dependencies(dependencies), name(n), _availableImg(0),
      _img1(480, 640, CV_8UC3), _img2(480, 640, CV_8UC3),
       _params(),
      _lockParams(), _lockImgs(),
      _pipeline(nullptr),
      monitor_scale(1), rhio_initialized(false),
      warningExecutionTime(0.01){
  _defaultRoi = false;
  setParameters();
}

Filter::~Filter() {
  if (display) {
    cv::destroyWindow(name);
  }
}

const std::string &Filter::getName() const { return name; }

SafePtr<const cv::Mat> Filter::getImg() const {
  _lockImgs.lock();
  if (_availableImg == 0) {
    return SafePtr<const cv::Mat>(_img1, _lockImgs, true);
  } else {
    return SafePtr<const cv::Mat>(_img2, _lockImgs, true);
  }
}

void Filter::addDependency(const std::string &name) {
  _dependencies.push_back(name);
}

SafePtr<const Filter::Parameters> Filter::params() const {
  return SafePtr<const Filter::Parameters>(_params, _lockParams);
}
SafePtr<Filter::Parameters> Filter::params() {
  return SafePtr<Filter::Parameters>(_params, _lockParams);
}

Utils::CameraState &Filter::getCS() {
  if (_pipeline == NULL) {
    throw std::runtime_error(
        "Requesting CameraState in a filter not attached to a pipeline");
  }
  Utils::CameraState *cs = _pipeline->getCameraState();
  if (cs == NULL) {
    throw std::runtime_error("Invalid camera state");
  } else {
    return *cs;
  }
}

Pipeline *Filter::getPipeline() {
  if (_pipeline == NULL) {
    throw std::runtime_error(
        "Requesting CameraState in a filter not attached to a pipeline");
  }
  return _pipeline;
}

void Filter::initWindow() {
  if (!display) {
    return;
  }

  // Create Opencv window for image and parameters
  cv::namedWindow(name, CV_WINDOW_NORMAL | CV_WINDOW_KEEPRATIO);

  // Set up and initialize mouse callback
  cv::setMouseCallback(
      name, [](int event, int x, int y, int, void *param) -> void {
        Filter *filter = (Filter *)param;
        if (event != cv::EVENT_LBUTTONDOWN) {
          return;
        }
        // Print coordinate, pixel value and world estimated position
        cv::Vec3b pixel = filter->getImg()->at<cv::Vec3b>(y, x);
        Eigen::Vector3d ball_pos;
        cv::Point2f ball_center_in_img = cv::Point2f(x,y);
        int radiusMin, radiusMax;
        double imgWidth = filter->img().cols;
        double imgHeight = filter->img().rows;
        std::cout << "filter->getCS()->timestamp" << filter->getCS().getTimeStampDouble()
                  << std::endl;
        ball_pos = filter->getCS().ballInfoFromPixel(ball_center_in_img,
                                                     imgWidth, imgHeight,
                                                     &radiusMin, &radiusMax);

        int B = (int)pixel[0];
        int G = (int)pixel[1];
        int R = (int)pixel[2];
        int Y = R * 0.299000 + G * 0.587000 + B * 0.114000;
        // http://softpixel.com/~cwright/programming/colorspace/yuv/
        //int U = R * -0.168736 + G * -0.331264 + B * 0.500000 + 128;
        //int V = R * 0.500000 + G * -0.418688 + B * -0.081312 + 128;
        // Wikipedia
        int U = R * -0.14713 + G * -0.28886 + B * 0.436 + 128;
        int V = R * 0.615 + G * -0.51499 + B * -0.10001 + 128;


        cv::Point2f undistorded;
        filter->getCS().undistortPoint(x, y, imgWidth, imgHeight,
                                       undistorded);
        // Code taken from ballInfoFromPixel
        Eigen::Vector2d pixelInLeph;//Leph limits are [-1,1]
        pixelInLeph.x() = ((double)(undistorded.x - imgWidth / 2.0) * 2.0 / imgWidth);
        pixelInLeph.y() = ((double)(undistorded.y - imgHeight / 2.0) * 2.0 / imgHeight);
        std::cout << "CLICK x=" << x << " y=" << y << std::endl;
        std::cout << " -> undistorded: "
                  << undistorded.x << ", " << undistorded.y << std::endl;
        std::cout << " ->pixelInLeph: "  << pixelInLeph.transpose() << std::endl;
        std::cout << " --> BGR[" << B << ",";
        std::cout << G << ",";
        std::cout << R << "]";
        
        std::cout << " --> YUV[" << Y << ",";
        std::cout << V << ",";
        std::cout << U << "]" << std::endl;
        std::cout << "Note :  YUV format is YCrCb" << std::endl;

        std::cout << "-> pos: " << ball_pos.transpose() << std::endl
                  << "-> radiusMin: " << radiusMin << std::endl
                  << "-> radiusMax: " << radiusMax << std::endl;
        // Draw radiusMin and radiusMax circles on the image
        for (int radius : {radiusMin, radiusMax}) {
          cv::circle(filter->cachedImg(),
                     ball_center_in_img,
                     radius,
                     cv::Scalar(255,0,0), 2);
        }
        filter->displayCurrent();
      }, this);
}

void Filter::update() {
  throw std::runtime_error("Update is not implemented for filter: " +
                           getClassName());
}

void Filter::previous() {
  throw std::runtime_error("Previous is not implemented for filter: " +
                           getClassName());
}

void Filter::runStep(UpdateType updateType) {
  Benchmark::open(name);
  // List of filters name
  // cout << getName() << endl;
  Benchmark::open("ImportFromRhio");
  importFromRhIO("Vision/");
  Benchmark::close("ImportFromRhio");
  // Call virtual implementation
  try {
    switch (updateType) {
    case forward:
      process();
      break;
    case backward:
      previous();
      break;
    case steady:
      update();
      break;
    default:
      std::cout << "Unhandled type of step in filter" << std::endl;
    }
  } catch (const cv::Exception &exc) {
    Benchmark::close(name.c_str());
    std::ostringstream oss;
    oss << "Filter '" << name << "' of class '" << getClassName()
        << "' failed to update throwing error:" << std::endl
        << exc.what();
    throw std::runtime_error(oss.str());
  } catch (...) {
    Benchmark::close(name.c_str());
    throw;
  }

  Benchmark::open("PublishToRhio");
  publishToRhIO("Vision/");
  Benchmark::close("PublishToRhio");
  // Check for dirty parameters TODO (still valid?)
  // --> params can no longer be changed dynamically, this solved some low level
  // issues with the new camera.
  // params()->checkDirty();

  // Switch image double buffer
  _lockImgs.lock();
  if (_availableImg == 0) {
    _availableImg = 1;
  } else {
    _availableImg = 0;
  }
  _lockImgs.unlock();

  /// Show output image if enabled
  if (display) displayCurrent();

  // Close filter benchmark
  double filterTime = Benchmark::close(name.c_str());
  if (filterTime > warningExecutionTime) {
    out.warning("Filter '%s' took %f ms for step", name.c_str(), filterTime * 1000);
  }
}

const Filter &Filter::getDependency(const std::string &name) const {
  if (_pipeline == nullptr) {
    throw std::logic_error("Filter not registered in pipeline");
  }
  auto it = find(_dependencies.begin(), _dependencies.end(), name);
  if (it == _dependencies.end()) {
    throw std::logic_error("Filter dependency unknown : looking for " + name +
                           " in " + getName());
  }

  return _pipeline->get(*it);
}

void Filter::initRhIO(const std::string &path) {
  // Lazy init
  if (rhio_initialized)
    return;

  rhio_initialized = true;
  rhio_path = path;

  std::string filterPath = path + getName();
  RhIO::Root.newChild(filterPath);
  RhIO::IONode &node = RhIO::Root.child(filterPath);

  // Declare variable if they have not been declared yet
  try {
    // This will fail if node do not exist yet
    node.getFloat("monitor_scale");
  } catch (const std::logic_error &err) {
    node.newFloat("monitor_scale")
        ->defaultValue(monitor_scale)
        ->minimum(0.01)
        ->maximum(10);
    // Advertise format
    node.newFrame("out", "Output frame of the filter '" + getName() + "'",
                  RhIO::FrameFormat::BGR);
  }
}

void Filter::importFromRhIO(const std::string &path) {
  initRhIO(path);
  std::string filterPath = path + getName();
  RhIO::IONode &node = RhIO::Root.child(filterPath);
  // Updating monitor scale
  monitor_scale = node.getValueFloat("monitor_scale").value;
  // Updating parameters
  for (const auto &param : params()->params<ParamInt>()) {
    if (!param.second.importFromRhIO()) {
      continue;
    }
    if (node.getValueType(param.first) != RhIO::NoValue) {
      param.second.value = node.getInt(param.first);
    }
  }
  for (const auto &param : params()->params<ParamFloat>()) {
    if (!param.second.importFromRhIO()) {
      continue;
    }
    if (node.getValueType(param.first) != RhIO::NoValue) {
      param.second.value = node.getFloat(param.first);
    }
  }
}

void Filter::publishToRhIO(const std::string &path) {
  initRhIO(path);
  std::string filterPath = path + getName();
  RhIO::IONode &node = RhIO::Root.child(filterPath);

  if (node.frameIsStreaming("out")) {
    // Publish frame to RhIO
    int monitor_width = (int)(monitor_scale * img().cols);
    int monitor_height = (int)(monitor_scale * img().rows);
    // Ensure that image is on three channels
    cv::Mat tmp_img1;
    if (img().channels() == 1) {
      cv::cvtColor(img(), tmp_img1, CV_GRAY2BGR);
    } else {
      tmp_img1 = img();
    }
    // Resize image if necessary
    cv::Mat tmp_img2(monitor_height, monitor_width, CV_8UC3);
    cv::resize(tmp_img1, tmp_img2, cv::Size(monitor_width, monitor_height));
    size_t size = 3 * monitor_height * monitor_width;
    node.framePush("out", monitor_width, monitor_height, tmp_img2.data, size);
  }

  // Publish parameters to RhIO if they have never been published
  for (const auto &param : params()->params<ParamInt>()) {
    if (node.getValueType(param.first) == RhIO::NoValue) {
      node.newInt(param.first)
          ->defaultValue(param.second.value)
          ->minimum(param.second.min)
          ->maximum(param.second.max);
    }
  }
  for (const auto &param : params()->params<ParamFloat>()) {
    if (node.getValueType(param.first) == RhIO::NoValue) {
      node.newFloat(param.first)
          ->defaultValue(param.second.value)
          ->minimum(param.second.min)
          ->maximum(param.second.max);
    }
  }
}

const Filter &Filter::getDependency() const {
  for (const auto &it : _dependencies) {
    return getDependency(it);
  }
  throw std::logic_error("Filter no dependency");
}

cv::Mat &Filter::img() {
  if (_availableImg == 0) {
    return _img2;
  } else {
    return _img1;
  }
}

cv::Mat &Filter::cachedImg() {
  if (_availableImg == 0) {
    return _img1;
  } else {
    return _img2;
  }
}

unsigned long Filter::now() const {
  return std::chrono::duration_cast<std::chrono::milliseconds>(
             std::chrono::steady_clock::now().time_since_epoch()).count();
}

void Filter::waitDependencies() const {
  // No wait of no dependency
  if (_dependencies.size() == 0) {
    return;
  }
}

Json::Value Filter::toJson() const {
  Json::Value v;
  v["className"] = getClassName();
  v["name"] = name;
  v["display"] = display;
  v["dependencies"] = vector2Json(_dependencies);
  v["paramInts"] = parameters2Json<int>();
  v["paramFloats"] = parameters2Json<float>();
  v["warningExecutionTime"] = warningExecutionTime;
  return v;
}

void Filter::fromJson(const Json::Value & v, const std::string & dir_name) {
  // First set parameters
  setParameters();
  try {
    // Then update content from json data
    rhoban_utils::tryRead(v,"name",&name);
    rhoban_utils::tryRead(v,"display",&display);;
    // Read dependencies
    rhoban_utils::tryReadVector(v, "dependencies", &_dependencies);
    // Checking dependencies size
    if (_dependencies.size() != (size_t)expectedDependencies()) {
      std::ostringstream oss;
      oss << "Filter::fromJson: '" << getName() << "' of class '"
          << getClassName() << "': invalid number of dependencies ("
          << _dependencies.size() << " received, " << expectedDependencies()
          << " expected)";
      throw rhoban_utils::JsonParsingError(oss.str());
    }
    // Read Parameters
    tryParametersUpdate<int  >(v, "paramInts");
    tryParametersUpdate<float>(v, "paramFloats");
  } catch (const rhoban_utils::JsonParsingError & err) {
    std::ostringstream oss;
    oss << err.what() << " in filter with name '" << name << "' of class '" << getClassName() << "'";
    throw rhoban_utils::JsonParsingError(oss.str());
  }
  rhoban_utils::tryRead(v, "warningExecutionTime", &warningExecutionTime);
  // Init display if required
  initWindow();
  // Publish parameters to RhIO
  publishToRhIO("Vision/");
}


const std::vector<std::pair<float, cv::RotatedRect>> &Filter::getRois() const {
  return _rois;
}

const std::vector<std::pair<float, Circle>> &Filter::getCircleRois() const {
  return _circleRois;
}

void Filter::addRoi(float quality, cv::RotatedRect _rect) {
  if (_defaultRoi) {
    _defaultRoi = false;
    _rois.clear();
  }
  // std::pair<cv::RotatedRect , float> duo(_rect, quality);
  // _rois.insert(duo);
  _rois.push_back({quality, _rect});
}

void Filter::addRoi(float quality, Circle _circle) {
  // _circleRois.insert(std::pair<Circle, float >(_circle, quality));
  _circleRois.push_back({quality, _circle});
  // _circleRois[_circle] = quality;
}

void Filter::clearRois() {
  _rois.clear();
  _circleRois.clear();
}

void Filter::displayCurrent() {
    Benchmark::open("display");

    // Computing current image
    cv::Mat & current_img = cachedImg();

    // TODO : make it cleaner, with values configurable outside the code
    // Resizing the windows to fit better their images
    float higherSize = 320.0; // Other side will be 240 if we're in 3/4
    int width, height;
    if (current_img.cols == 0 || current_img.rows == 0) {
      width = 320;
      height = 320;
    } else {
      if (current_img.cols > current_img.rows) {
        width = higherSize;
        height = current_img.rows / (current_img.cols / higherSize);
      } else {
        height = higherSize;
        width = current_img.cols / (current_img.rows / higherSize);
      }
    }

    cv::resizeWindow(name, width, height);
    cv::imshow(name, current_img);

    /*
     * Positionning the images. TODO : make it cleaner, with values configurable
     * outside the code
     */
    if (std::find(listOfPresentWindows.begin(), listOfPresentWindows.end(),
                  name) == listOfPresentWindows.end()) {
      // First time with ask to print this
      int nbElements = listOfPresentWindows.size();
      int xOffset = 50;
      int yOffset = 50;
      int widthImg = 250;
      int heightImg = 360;
      int widthScreen = 1920;
      int xIndex = nbElements % (widthScreen / widthImg);
      int yIndex = nbElements / (widthScreen / widthImg);
      cv::moveWindow(name, xIndex * widthImg + xOffset,
                     yIndex * heightImg + yOffset);
      Filter::listOfPresentWindows.push_back(name);
    }

    Benchmark::close("display");
}

}
