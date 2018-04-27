#pragma once

#include "Filters/Filter.hpp"

#include <opencv2/opencv.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <string>
#include <aruco.h>

namespace Vision {
namespace Filters {

/**
 * TagsDetector
 *
 */
class TagsDetector : public Filter {
public:
  TagsDetector();



  // Json stuff
  // virtual void fromJson(const Json::Value & v, const std::string & dir_name);
  // virtual Json::Value toJson() const;
  virtual std::string getClassName() const override { return "TagsDetector"; }

  virtual void setParameters() override;
  virtual int expectedDependencies() const override;

  std::vector<aruco::Marker> getDetectedMarkers() const;

protected:
  /**
   * @Inherit
   */
  virtual void process() override;

private:
  ParamFloat thresholdParam1;
  ParamFloat thresholdParam2;
  /// Try all parameters1 in [param1 - paramRange1, param1 + paramRange1]
  /// Keep it to 0 when using ADPT_THRES (default), because only 'odd' values
  /// for param1 are considered (thus half of the computation time is useless)
  ParamInt thresholdParamRange1;
  
  ParamFloat minSize;
  ParamFloat maxSize;
  ParamInt minSize_pix;
  ParamInt subpix_wsize;
  /// Size of the markers in [m]
  ParamFloat markerSize;

  /// 0 -> no output and no copy of the image
  /// 1 -> Display markers and cubes
  /// 2 -> Verbose output
  ParamInt debugLevel;

  /// Since aruco tag execution is expensive, it is only executed every
  /// 'period' images
  ParamInt period;

  // Detection parameters
  aruco::MarkerDetector::Params detectionParam;
  // Camera parameters (distortion and camera matrix)
  aruco::CameraParameters CamParam;
  aruco::MarkerDetector MDetector;

  std::vector<aruco::Marker>  Markers;

  int periodCounter;

};
}
}
