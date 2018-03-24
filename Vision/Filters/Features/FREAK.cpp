#include <stdexcept>
#include "Filters/Features/FREAK.hpp"
#include <opencv2/legacy/legacy.hpp>
#include "rhoban_utils/timing/time_stamp.h"
#include "Utils/RotatedRectUtils.hpp"
#include "rhoban_utils/logging/logger.h"
#include "CameraState/CameraState.hpp"

static rhoban_utils::Logger out("FREAK");

namespace Vision {
namespace Filters {

void FREAK::deserialize() {
  cv::FileStorage fs(featurePath, cv::FileStorage::READ);
  if (fs.isOpened()) {
    int size = fs["nbkeys"];
    for (int i = 0; i < size; i++) {
      cv::FileNode kptFileNode = fs["keypoints_" + std::to_string(i)];
      std::vector<cv::KeyPoint> kp;
      cv::read(kptFileNode, kp);
      _keypoints_set.push_back(kp);
    }
    fs["descriptors"] >> _descriptors_set;
    fs["pairs"] >> _pairs;
    fs.release();

    cv::FREAK _extractor(true, true, 22.0f, 4, _pairs);
  } else {
    std::ostringstream oss;
    oss << "Error opening learning file: " << featurePath << endl;
    throw std::runtime_error(oss.str());
  }
}

void FREAK::setParameters() {

  matchThreshold = ParamInt(2, 0, 20, ParameterType::PARAM);
  distThreshold = ParamInt(100, 0, 200, ParameterType::PARAM);
  keypointSensibility = ParamInt(70, 0, 150, ParameterType::PARAM);
  maxDistanceInCm = ParamInt(100, 0, 500, ParameterType::PARAM);

  params()->define<std::vector<double>>("ballsX", ballsX);
  params()->define<std::vector<double>>("ballsY", ballsY);
  params()->define<std::vector<double>>("ballsRadius", ballsRadius);
  params()->define<int>("imgWidth", 640);
  params()->define<int>("imgHeight", 480);
  params()->define<ParamInt>("matchThreshold", &matchThreshold);
  params()->define<ParamInt>("distThreshold", &distThreshold);
  params()->define<ParamInt>("keypointSensibility", &keypointSensibility);
  params()->define<ParamInt>("maxDistanceInCm", &maxDistanceInCm);

  tilt = ParamFloat(0, -10, 90, ParameterType::OUTPUT);
  qTilt = ParamFloat(0, 0, 1, ParameterType::OUTPUT);

  // Entry params
  params()->define<ParamFloat>("tilt", &tilt);
  params()->define<ParamFloat>("qTilt", &qTilt);
}
void FREAK::process() {
  _normalizedROI.clear();
  ballsX.clear();
  ballsY.clear();
  ballsRadius.clear();

  // If the tilt is to high, we're not even processing the filter
  Utils::CameraState &cs = getCS();
  float angleToGround = tilt + abs(cs.getVertApertureDeg() / 2);
  cout << "cs.getHeight() = " << cs.getHeight() << endl;
  float tiltLimit = 90 - (180 / M_PI) * atan(maxDistanceInCm / cs.getHeight());
  cout << "angleToGround = " << angleToGround << endl;
  cout << "tiltLimit = " << tiltLimit << endl;
  if (angleToGround > tiltLimit) {
    return;
  }

  std::string srcName = _dependencies[0];
  std::string roiManagerName = _dependencies[1];
  cv::Mat srcImg = *(getDependency(srcName).getImg());
  img() = srcImg.clone();
  cv::Mat channels[3];
  cv::split(srcImg, channels);
  srcImg = channels[2];
  std::vector<std::pair<float, cv::RotatedRect>> rois =
      getDependency(roiManagerName).getRois();
  std::vector<std::pair<float, Circle>> circleRois =
      getDependency(roiManagerName).getCircleRois();

  //    cv::Rect r(0, 0, srcImg.cols, srcImg.rows);
  // cv::Mat m(srcImg.size(), 0, cv::Scalar(0));
  //_normalizedROI.push_back({r, m});

  try {
    // Normalizing rois
    for (auto circleRoi : circleRois) {
      normalizeROI(circleRoi.second);
    }
  } catch (const cv::Exception &exc) {
    out.warning("Exception while Normalizing Circles");
    out.warning(exc.what());
  }
  try {
    for (auto smallRoi : rois) {
      normalizeROI(smallRoi.second);
    }
  } catch (const cv::Exception &exc) {
    out.warning("Exception while Normalizing smallRois");
    out.warning(exc.what());
  }

  int i = 0;
  for (auto roi : _normalizedROI) {
    if (roi.first.width < 1 || roi.first.height < 1) {
      continue;
    }
    cv::Mat src = cv::Mat(srcImg, roi.first);
    cv::Mat mask = roi.second;

    // detection
    cv::FastFeatureDetector detector(keypointSensibility);
    std::vector<cv::KeyPoint> curr_keypoints;
    detector.detect(src, curr_keypoints);

    // extraction
    cv::Mat curr_descriptor;
    if (curr_keypoints.size() > 0) {
      _extractor.compute(src, curr_keypoints, curr_descriptor);
      cv::drawKeypoints(src, curr_keypoints, src, cv::Scalar::all(-1),
                        cv::DrawMatchesFlags::DEFAULT);

      // match
      std::vector<cv::DMatch> matches;
#if CV_SSSE3
      cv::BruteForceMatcher<cv::HammingSeg<30, 4>> matcher;
#else
      cv::BruteForceMatcher<cv::Hamming> matcher;
#endif

      matcher.add(_descriptors_set);
      matcher.match(curr_descriptor, matches);
      double max_dist = 0;
      double min_dist = 100;
      for (int i = 0; i < curr_descriptor.rows; i++) {
        double dist = matches[i].distance;
        if (dist < min_dist)
          min_dist = dist;
        if (dist > max_dist)
          max_dist = dist;
      }
      int sumX = 0, sumY = 0, n = 0;
      auto tl = roi.first.tl();
      for (size_t i = 0; i < matches.size(); i++) {
        if (matches[i].distance < distThreshold) { // TODO maybe work on
                                                   // distance if too many false
                                                   // positives
          auto point = curr_keypoints[matches[i].queryIdx].pt;
          sumX += point.x + tl.x;
          sumY += point.y + tl.y;
          n++;
          cv::circle(img(), cv::Point(point.x + tl.x, point.y + tl.y),
                     (max_dist / (matches[i].distance + 1)) * 1.5,
                     cv::Scalar(255, 0, 0), -1);

          // std::cout << "dist " << matches[i].distance << std::endl;

          // ballsX.push_back(point.x + tl.x);
          // ballsY.push_back(point.y + tl.y);
          // ballsRadius.push_back(-1);
        }
      }
      if (n > matchThreshold) { // TODO tune number of features allowed
        double meanX = sumX / n;
        double meanY = sumY / n;
        ballsX.push_back(meanX);
        ballsY.push_back(meanY);
        cv::circle(img(), cv::Point(meanX, meanY), 5, cv::Scalar(0, 0, 255),
                   -1);
        ballsRadius.push_back(-1);
      }
    }
    // cv::imshow(std::to_string(i), src);

    i++;
  }
  params()->set<std::vector<double>>("ballsX") = ballsX;
  params()->set<std::vector<double>>("ballsY") = ballsY;
  params()->set<std::vector<double>>("ballsRadius") = ballsRadius;
  // imshow("",mask);
}

void FREAK::normalizeROI(const cv::RotatedRect &roi) {
  cv::Rect boundingRect = cropRect(toRect(roi));
  cv::Mat rotatedRoiMask(cv::Size(boundingRect.width, boundingRect.height), 0,
                         cv::Scalar(0));
  cv::RotatedRect rectForMask(
      cv::Point(boundingRect.width / 2, boundingRect.height / 2), roi.size,
      roi.angle);
  drawRotatedRectangle(rotatedRoiMask, rectForMask, cv::Scalar(255), -1);
  _normalizedROI.push_back({boundingRect, rotatedRoiMask});
}

void FREAK::normalizeROI(const Circle &roi) {
  cv::Point center(roi.getCenter().x, roi.getCenter().y);
  int width = 2 * roi.getRadius();
  int height = width;
  cv::Rect boundingRect(center.x - width / 2, center.y - height / 2, width,
                        height);
  boundingRect = cropRect(boundingRect);
  cv::Mat circleMask(cv::Size(boundingRect.width, boundingRect.height), 0,
                     cv::Scalar(0));
  Circle circleForMask(boundingRect.width / 2, boundingRect.height / 2,
                       roi.getRadius());
  cv::circle(circleMask, cv::Point(circleForMask.getCenter().x,
                                   circleForMask.getCenter().y),
             (int)circleForMask.getRadius(), cv::Scalar(255), -1, 8, 0);
  _normalizedROI.push_back({boundingRect, circleMask});
}
cv::Rect FREAK::cropRect(const cv::Rect &roi) {
  cv::Rect newROI = cv::Rect(roi);
  if (newROI.x < 0) {
    newROI.width += newROI.x;
    newROI.x = 0;
  }
  if (newROI.y < 0) {
    newROI.height += newROI.y;
    newROI.y = 0;
  }
  int maxHeight = img().rows - newROI.y;
  int maxWidth = img().cols - newROI.x;
  if (newROI.width > maxWidth)
    newROI.width = maxWidth;
  if (newROI.height > maxHeight)
    newROI.height = maxHeight;

  return newROI;
}
cv::Rect FREAK::toRect(const cv::RotatedRect &rotatedRect) {
  if (rotatedRect.angle == 0) {
    cv::Rect rect(rotatedRect.center.x - rotatedRect.size.width / 2,
                  rotatedRect.center.y - rotatedRect.size.height / 2,
                  rotatedRect.size.width, rotatedRect.size.height);
    return rect;
  } else {
    return rotatedRect.boundingRect();
  }
}
void FREAK::fromJson(const Json::Value & v, const std::string & dir_name) {
  Filter::fromJson(v, dir_name);
  rhoban_utils::tryRead(v,"featurePath",&featurePath);
  deserialize();
}

Json::Value FREAK::toJson() const {
  Json::Value v = Filter::toJson();
  v["featurePath"] = featurePath;
  return v;
}
}
}
