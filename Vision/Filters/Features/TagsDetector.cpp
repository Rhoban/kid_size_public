#include "Filters/Features/TagsDetector.hpp"

#include "rhoban_utils/logging/logger.h"
#include "rhoban_utils/timing/benchmark.h"

#include "CameraState/CameraState.hpp"

#include <vector>
#include <iostream>

using namespace std;
using ::rhoban_utils::Benchmark;

static rhoban_utils::Logger out("TagsDetector");

namespace Vision {
namespace Filters {

TagsDetector::Marker::Marker()
  : id(-1), size(-1), rvec(), tvec()
{}

TagsDetector::Marker::Marker(int id, float size, const std::vector<cv::Point2f> & corners,
                             const cv::Vec3d & rvec, const cv::Vec3d & tvec)
  : id(id), size(size), corners(corners), rvec(rvec), tvec(tvec)
{}

TagsDetector::TagsDetector() : Filter("TagsDetector")
{
  periodCounter = 0;
}


void TagsDetector::setParameters() {
  adaptiveThreshConstant = ParamFloat(7, 3, 23);
  params()->define<ParamFloat>("adaptiveThreshConstant", &adaptiveThreshConstant);
  adaptiveThreshWinSizeMin = ParamInt(7, 3, 23);
  params()->define<ParamInt>("adaptiveThreshWinSizeMin", &adaptiveThreshWinSizeMin);
  adaptiveThreshWinSizeMax = ParamInt(7, 3, 23);
  params()->define<ParamInt>("adaptiveThreshWinSizeMax", &adaptiveThreshWinSizeMax);
  adaptiveThreshWinSizeStep = ParamInt(7, 3, 23);
  params()->define<ParamInt>("adaptiveThreshWinSizeStep", &adaptiveThreshWinSizeStep);
  
 //marker size in m
  markerSize = ParamFloat(0.09, 0, 1.0);
  params()->define<ParamFloat>("markerSize", &markerSize);
  
  debugLevel = ParamInt(0, 0, 2);
  params()->define<ParamInt>("debugLevel", &debugLevel);
  period = ParamInt(1, 1, 100);
  params()->define<ParamInt>("period", &period);
}
  
void TagsDetector::process() {
  markers.clear();
  
  // End function if we have not reached period
  periodCounter++;
  if (periodCounter < period) {
    return;
  }
  periodCounter = 0;

  const cv::Mat & srcImg = *(getDependency().getImg());

  // Importing parameters from rhio
  // Adaptive threshold is used since it is note modified
  detectorParameters->adaptiveThreshConstant = adaptiveThreshConstant;
  detectorParameters->adaptiveThreshWinSizeMin = adaptiveThreshWinSizeMin;
  detectorParameters->adaptiveThreshWinSizeMax = adaptiveThreshWinSizeMax;
  detectorParameters->adaptiveThreshWinSizeStep = adaptiveThreshWinSizeStep;

  // Copying image if necessary
  if (debugLevel > 0) {
    img() = srcImg.clone();
  } else {
    img() = srcImg;
  }
  Benchmark::open("Loading dic");
  cv::Ptr<cv::aruco::Dictionary> dic = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
  Benchmark::close("Loading dic");

  Benchmark::open("Detecting Tags");
  std::vector<int> markerIds;
  std::vector<std::vector<cv::Point2f>> markerCorners;
  cv::aruco::detectMarkers(img(), dic, markerCorners, markerIds, detectorParameters);
  Benchmark::close("Detecting Tags");
  
  Benchmark::open("Estimating poses");
  std::vector<cv::Vec3d> rvecs, tvecs;
  const Leph::CameraModel & cam_model = getCS().getCameraModel();
  cv::aruco::estimatePoseSingleMarkers(markerCorners, markerSize,
                                       cam_model.getCameraMatrix(),cam_model.getDistortionCoeffs(),
                                       rvecs, tvecs);
  Benchmark::close("Estimating poses");
  
  Benchmark::open("Gathering markers");
  for (size_t i = 0; i < markerIds.size(); i++) {
    markers.push_back(Marker(markerIds[i], markerSize, markerCorners[i], rvecs[i], tvecs[i]));
  }
  Benchmark::close("Gathering markers");

  if (debugLevel >= 1) {
    Benchmark::open("Drawing Tags");
    cv::aruco::drawDetectedMarkers(img(), markerCorners, markerIds, cv::Scalar(0,0,255));    
    Benchmark::close("Drawing Tags");
  }

}

const std::vector<TagsDetector::Marker> & TagsDetector::getDetectedMarkers() const
{
  return markers;
}

int TagsDetector::expectedDependencies() const {
  return 1;
}

}
}
