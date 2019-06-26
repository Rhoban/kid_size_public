#pragma once

#include "GroundTruthProvider.hpp"

#include <random>

namespace Vision
{
namespace Filters
{
/**
 * dependency[0]: source image (used to extract patches)
 * dependency[1]: use as roi provider
 */
class ROIBasedGTP : public GroundTruthProvider
{
public:
  ROIBasedGTP();
  std::string getClassName() const override;
  int expectedDependencies() const override;

protected:
  std::vector<cv::Rect_<float>> generateROIs() override;
  const cv::Mat& getSourceImg() override;
};

}  // namespace Filters
}  // namespace Vision
