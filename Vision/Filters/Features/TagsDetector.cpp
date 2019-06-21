#include "Filters/Features/TagsDetector.hpp"

#include "rhoban_utils/logging/logger.h"
#include "rhoban_utils/timing/benchmark.h"

#include "CameraState/CameraState.hpp"

#include <vector>
#include <iostream>

using namespace std;
using ::rhoban_utils::Benchmark;

static rhoban_utils::Logger out("TagsDetector");

namespace Vision
{
namespace Filters
{
TagsDetector::Marker::Marker() : id(-1), size(-1), rvec(), tvec()
{
}

TagsDetector::Marker::Marker(int id, float size, const std::vector<cv::Point2f>& corners, const cv::Vec3d& rvec,
                             const cv::Vec3d& tvec)
  : id(id), size(size), corners(corners), rvec(rvec), tvec(tvec)
{
}

cv::Point2f TagsDetector::Marker::getCenter() const
{
  cv::Point2f center(0, 0);
  for (const cv::Point2f corner : corners)
  {
    center += corner / (float)corners.size();
  }
  return center;
}

TagsDetector::TagsDetector()
  : Filter("TagsDetector")
  , detectorParameters(new cv::aruco::DetectorParameters())
  , markersData({ "image_id", "marker_id", "pixel_x", "pixel_y" },
                { { "image_id", {} }, { "marker_id", {} }, { "pixel_x", {} }, { "pixel_y", {} } })
{
  periodCounter = 0;
}

void TagsDetector::setParameters()
{
  adaptiveThreshConstant = ParamFloat(7, 3, 23);
  params()->define<ParamFloat>("adaptiveThreshConstant", &adaptiveThreshConstant);
  adaptiveThreshWinSizeMin = ParamInt(7, 3, 23);
  params()->define<ParamInt>("adaptiveThreshWinSizeMin", &adaptiveThreshWinSizeMin);
  adaptiveThreshWinSizeMax = ParamInt(7, 3, 23);
  params()->define<ParamInt>("adaptiveThreshWinSizeMax", &adaptiveThreshWinSizeMax);
  adaptiveThreshWinSizeStep = ParamInt(7, 3, 23);
  params()->define<ParamInt>("adaptiveThreshWinSizeStep", &adaptiveThreshWinSizeStep);
  cornerRefinementMaxIterations = ParamInt(50, 20, 100);
  params()->define<ParamInt>("cornerRefinementMaxIterations", &cornerRefinementMaxIterations);
  cornerRefinementMinAccuracy = ParamFloat(0.01, 0.0001, 1);
  params()->define<ParamFloat>("cornerRefinementMinAccuracy", &cornerRefinementMinAccuracy);
  cornerRefinementWinSize = ParamInt(3, 1, 10);
  params()->define<ParamInt>("cornerRefinementWinSize", &cornerRefinementWinSize);

  // marker size in m
  markerSize = ParamFloat(0.09, 0, 1.0);
  params()->define<ParamFloat>("markerSize", &markerSize);

  debugLevel = ParamInt(0, 0, 2);
  params()->define<ParamInt>("debugLevel", &debugLevel);
  period = ParamInt(1, 1, 100);
  params()->define<ParamInt>("period", &period);
  isWritingData = ParamInt(0, 0, 1);
  params()->define<ParamInt>("isWritingData", &isWritingData);
  refine = ParamInt(1, 0, 1);
  params()->define<ParamInt>("refine", &refine);
}

void TagsDetector::process()
{
  markers.clear();

  // End function if we have not reached period
  periodCounter++;
  if (periodCounter < period)
  {
    return;
  }
  periodCounter = 0;

  const cv::Mat& srcImg = *(getDependency().getImg());

  // Importing parameters from rhio
  // Adaptive threshold is used since it is note modified
  detectorParameters->adaptiveThreshConstant = adaptiveThreshConstant;
  detectorParameters->adaptiveThreshWinSizeMin = adaptiveThreshWinSizeMin;
  detectorParameters->adaptiveThreshWinSizeMax = adaptiveThreshWinSizeMax;
  detectorParameters->adaptiveThreshWinSizeStep = adaptiveThreshWinSizeStep;
  detectorParameters->doCornerRefinement = refine;
  detectorParameters->cornerRefinementMaxIterations = cornerRefinementMaxIterations;
  detectorParameters->cornerRefinementMinAccuracy = cornerRefinementMinAccuracy;
  detectorParameters->cornerRefinementWinSize = cornerRefinementWinSize;

  // Copying image if necessary
  if (debugLevel > 0)
  {
    img() = srcImg.clone();
  }
  else
  {
    img() = srcImg;
  }
  Benchmark::open("Loading dic");
  cv::Ptr<cv::aruco::Dictionary> dic = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_100);
  Benchmark::close("Loading dic");

  Benchmark::open("Detecting Tags");
  std::vector<int> markerIds;
  std::vector<std::vector<cv::Point2f>> markerCorners;
  cv::aruco::detectMarkers(img(), dic, markerCorners, markerIds, detectorParameters);
  Benchmark::close("Detecting Tags");

  Benchmark::open("Estimating poses");
  std::vector<cv::Vec3d> rvecs, tvecs;
  const rhoban::CameraModel& cam_model = getCS().getCameraModel();
  cv::aruco::estimatePoseSingleMarkers(markerCorners, markerSize, cam_model.getCameraMatrix(),
                                       cam_model.getDistortionCoeffs(), rvecs, tvecs);
  Benchmark::close("Estimating poses");

  Benchmark::open("Gathering markers");
  for (size_t i = 0; i < markerIds.size(); i++)
  {
    markers.push_back(Marker(markerIds[i], markerSize, markerCorners[i], rvecs[i], tvecs[i]));
  }
  Benchmark::close("Gathering markers");

  if (debugLevel >= 1)
  {
    out.log("The number of tag is %d", markers.size());
  }
  if (debugLevel >= 1)
  {
    Benchmark::open("Drawing Tags");
    cv::aruco::drawDetectedMarkers(img(), markerCorners, markerIds, cv::Scalar(0, 0, 255));
    Benchmark::close("Drawing Tags");
  }

  if (isWritingData == 1)
  {
    Benchmark::open("Writing Tags");
    if (markers.size() > 0)
    {
      for (const Marker& m : markers)
      {
        cv::Point2f center = m.getCenter();
        std::map<std::string, std::string> row = { { "image_id", std::to_string(dumpCounter) },
                                                   { "marker_id", std::to_string(m.id) },
                                                   { "pixel_x", std::to_string(center.x) },
                                                   { "pixel_y", std::to_string(center.y) } };
        markersData.insertRow(row);
      }
    }
    dumpCounter++;
    // Save after every iteration (should be updated, but it would require to
    // take care of the case where the filter is closed/destroyed or if
    // dumpCounter is turned off)
    markersData.writeFile("tags_detector_data.csv");
    Benchmark::close("Writing Tags");
  }
  else
  {
    dumpCounter = 0;
    markersData.clearData();
  }
}

const std::vector<TagsDetector::Marker>& TagsDetector::getDetectedMarkers() const
{
  return markers;
}

int TagsDetector::expectedDependencies() const
{
  return 1;
}

}  // namespace Filters
}  // namespace Vision
