#pragma once

#include "Filters/Filter.hpp"

#include <opencv2/opencv.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <utility>
#include <string>
#include <vector>

namespace Vision {
namespace Filters {

/**
 * FREAK
 *
 */
class FREAK : public Filter {
public:
  typedef rhoban_geometry::Circle Circle;
  
  FREAK() : Filter("FREAK") {}

  /**
   * Initialization with filter name and asked frequency
   */
  FREAK(const std::string &name, const std::string &path,
        Frequency::type frequency = Frequency::Auto);

  void deserialize();
  void normalizeROI(const cv::RotatedRect &roi);
  void normalizeROI(const Circle &roi);
  cv::Rect cropRect(const cv::Rect &roi);
  cv::Rect toRect(const cv::RotatedRect &rotatedRect);

  // Json stuff
  virtual void fromJson(const Json::Value & v, const std::string & dir_name);
  virtual Json::Value toJson() const;
  virtual std::string getClassName() const override { return "FREAK"; }

  virtual void setParameters() override;
  virtual int expectedDependencies() const override { return 2; }

protected:
  /**
   * @Inherit
   */
  virtual void process() override;

private:
  std::vector<double> ballsX, ballsY, ballsRadius;
  std::vector<std::pair<cv::Rect, cv::Mat>> _normalizedROI;
  std::string featurePath;

  std::vector<std::vector<cv::KeyPoint>> _keypoints_set;
  std::vector<cv::Mat> _descriptors_set;
  std::vector<int> _pairs;
  cv::FREAK _extractor;

  ParamInt matchThreshold;
  ParamInt distThreshold;
  ParamInt keypointSensibility;
  ParamInt maxDistanceInCm;

  ParamFloat tilt, qTilt;
};
}
}
