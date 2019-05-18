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
  void setParameters() override;

protected:
  std::vector<cv::Rect_<float>> generateROIs() override;
  const cv::Mat& getSourceImg() override;

  /**
   * The number of patches generated using random numbers
   */
  ParamInt nbGeneratedPatches;

  /**
   * Maximal number of pixels of difference for Patches
   */
  ParamInt maxPosNoise;

  /**
   * Maximal difference of size ratio when generating Patches
   */
  ParamFloat maxRatioNoise;

  std::default_random_engine engine;
};

}  // namespace Filters
}  // namespace Vision
