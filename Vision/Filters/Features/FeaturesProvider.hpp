#pragma once

#include <hl_monitoring/field.h>

namespace Vision
{
namespace Filters
{

class FeaturesProvider
{
  
public:
  FeaturesProvider();
  virtual ~FeaturesProvider();

  const std::vector<cv::Point2f>& getBalls() const;
  const std::map<hl_monitoring::Field::POIType, std::vector<cv::Point2f>>& getPOIs() const;
  
  void clearAllFeatures();
  void pushBall(const cv::Point2f& img_pos);
  void pushPOI(hl_monitoring::Field::POIType type, const cv::Point2f& img_pos);
  
  
private:
  std::vector<cv::Point2f> ball_positions;
  std::map<hl_monitoring::Field::POIType, std::vector<cv::Point2f>> poi_positions;
  // TODO add storage for robot
  
  
};
  





  
}
}
