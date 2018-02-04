#pragma once

#include <opencv2/core/core.hpp>

#include "rhoban_utils/angle.h"

/**
 * TODO separate the notions of :
 * - img (camera image)
 * - topRobotView (From the top of the robot)
 * - topFieldView (From the top of the field)
 *
 * Different files should be used eventually
 */

namespace Vision {
namespace Localisation {

/**
 * Allow to draw/represent the belief of the ball position in the robot
 * basis
 */
class RobotBasis {
private:
  static cv::Point center(const cv::Mat &img);

public:
  static double maxDist;     // in cm
  static double robotHeight; // in cm

  static double getScale(const cv::Mat &img); // nb pixels by centimeter

  static void drawRobotView(cv::Mat &dst, const rhoban_utils::Angle &robotPan);
  void setHeight(double height);
  double getHeight(); // CM!

  /**
   * All the following is deprecated because it is an approximation of
   * the reality which consider that x-position in image has no effect
   * over the tilt, even when initial tilt is high, which is false.
   */

  // Convert a couple (pan,tilt) to a position in the image
  static cv::Point panTiltToImg(const cv::Mat &img, const rhoban_utils::Angle &pan,
                                const rhoban_utils::Angle &tilt);
  // Convert a point in the robot basis to the image
  static cv::Point robotToImg(const cv::Mat &img, double x, double y);
  static cv::Point robotToImg(const cv::Mat &img, const cv::Point p);
  // Convert a point in the robot basis to the pan-tilt coordinate
  // static void robotToPanTilt(const cv::Point & pos,
  //                           Angle & pan,
  //                           Angle & tilt);
};
}
}
