#include "RobotBasis.hpp"

#include "opencv2/imgproc/imgproc.hpp"

#include "CameraState/CameraState.hpp"

using namespace rhoban_utils;

namespace Vision {
namespace Localisation {

double RobotBasis::maxDist = 1200.0;

// FIXME: remove, use camerastate
double RobotBasis::robotHeight = 0.0; // WARNING

double RobotBasis::getScale(const cv::Mat &img) {
  // Calculating max scale according to the image height and width
  double scaleX = img.cols / (2 * maxDist);
  double scaleY = img.rows / (2 * maxDist);
  // Choosing lowest scale
  return scaleX > scaleY ? scaleY : scaleX;
}

cv::Point RobotBasis::center(const cv::Mat &img) {
  return cv::Point(img.cols / 2, img.rows / 2);
}

cv::Point RobotBasis::panTiltToImg(const cv::Mat &img, const Angle &pan,
                                   const Angle &tilt) {
  double tanTilt = tan(tilt);
  double dist = maxDist;
  if (tanTilt != 0)
    dist = robotHeight / tanTilt;
  return robotToImg(img, dist * cos(pan), dist * sin(pan));
}

cv::Point RobotBasis::robotToImg(const cv::Mat &img, double x, double y) {
  cv::Point offset((int)(-y * getScale(img)), (int)(-x * getScale(img)));
  return center(img) + offset;
}

// void RobotBasis::robotToPanTilt(const cv::Point & pos,
//                                Angle & pan, Angle & tilt)
//{
//  pan = Angle::fromXY(pos.x, pos.y);
//  double dist = sqrt(pos.x * pos.x + pos.y * pos.y);
//  tilt = Angle::fromXY(dist, robotHeight);
//}

void RobotBasis::drawRobotView(cv::Mat &dst, const Angle &robotPan) {
  // Field
  rectangle(dst, cv::Point(0, 0), cv::Point(dst.cols, dst.rows),
            cv::Scalar(0, 128, 0), CV_FILLED);
  // Robot at center
  cv::Point robotHalfSize((int)20 * getScale(dst), (int)20 * getScale(dst));
  rectangle(dst, center(dst) - robotHalfSize, center(dst) + robotHalfSize,
            cv::Scalar(0, 0, 0), CV_FILLED);
  // Thin circles for 1,2,4,8 meter
  for (int radius = 100; radius < maxDist * 1.44; radius *= 2) {
    circle(dst, center(dst), radius * getScale(dst), cv::Scalar(255, 255, 255),
           2);
  }
  // TODO: enable again but with the appropriate camerastate
  //      for (auto& dir : {-1,0,1}){
  //          double vectorLength = maxDist;
  //          double vectorThickness = 5 * getScale(dst);
  //          cv::Scalar color(128,0,0);
  //          Angle vecDir = robotPan + Angle(dir *
  //          Utils::CameraState::latAperture/2);
  //          if (dir == 0){
  //              vectorLength /= 10;
  //              vectorThickness *= 2;
  //              color = cv::Scalar(0,0,0);
  //          }
  //          cv::Point endVec = robotToImg(dst,
  //                                        vectorLength * cos(vecDir),
  //                                        vectorLength * sin(vecDir));
  //          line(dst, center(dst), endVec, color, vectorThickness);
  //      }
}
}
}
