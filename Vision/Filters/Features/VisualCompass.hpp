#pragma once

#include "Filters/Filter.hpp"
#include "CompassProvider.hpp"

#include <opencv2/opencv.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "Features/nonfree/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <string>
#include <vector>
#include <aruco.h> //FIXME annoying to include that juste to access the cameraparam struct

namespace Vision {
namespace Filters {

/**
 * VisualCompass
 *
 */
class VisualCompass : public CompassProvider {
public:

  VisualCompass();
  virtual std::string getClassName() const override { return "VisualCompass"; }

  virtual void setParameters() override;
  virtual int expectedDependencies() const override { return 1; }

  void getPoints(const std::vector<cv::KeyPoint>& kpts_train,
                 const std::vector<cv::KeyPoint>& kpts_query, std::vector<cv::Point2f>& pts_train,
                 std::vector<cv::Point2f>& pts_query,std::vector<cv::DMatch> & good_matches);
  void computeAngle(std::vector<cv::KeyPoint> keypoints,  const std::vector<cv::DMatch>& good_matches  );

  //useless (and overkill)
  void cameraPoseFromHomography(const cv::Mat& H, cv::Mat& pose);

  void sanitizeCoordinates(const std::vector<cv::Point2f> &in, std::vector<cv::Point2f> &out);
  void initSIFT();
  void matchSIFT(std::vector<cv::KeyPoint> &keypoints,  std::vector<cv::DMatch>& good_matches );

  void updateField();

  virtual void fromJson(const Json::Value & v, const std::string & dir_name) override;

protected:
  /**
   * @Inherit
   */
  virtual void process() override;


private:
  /// Should we update the field
  int active_field;

  aruco::CameraParameters CamParam;
  cv::Ptr<cv::FeatureDetector> detector;
  cv::Ptr<cv::DescriptorExtractor> descriptor_extractor;


  ParamFloat nndrRatio;
  ParamFloat panoScale;

  ParamInt knn_K;

  ParamInt debugLevel;
  ParamInt enabled;

  ParamInt fieldNumber;

  ParamFloat maskBelow;
  ParamFloat maskAbove;

  float prev_maskBelow;
  float prev_maskAbove;

  std::vector<cv::KeyPoint> keypoints_pano;
  cv::Mat descriptors_pano;
  cv::Ptr<cv::DescriptorMatcher> descriptorMatcher;
  cv::Ptr<cv::flann::Index> flannIndex;

  cv::Mat field; //FIXME, we should probably not store it
  cv::Mat field_mask;

  cv::Mat img_mask;

  cv::Mat img_debug;

  bool isHomographyGood;
  float hfov_rad;
  bool cameraparamsdone;
  float pixeltoangle_pano;
  float angletopixel_pano;
  float pixeltoangle;

  int width;
  int pano_width;
  int height;
  int pano_height;

};
}
}
