#include "ROIBasedGTP.hpp"

#include <Utils/ROITools.hpp>
#include <rhoban_utils/logging/logger.h>
#include "rhoban_random/tools.h"
#include <random>

static rhoban_utils::Logger logger("ROIBasedGTP");

namespace Vision
{
namespace Filters
{
ROIBasedGTP::ROIBasedGTP()
{
}

std::string ROIBasedGTP::getClassName() const
{
  return "ROIBasedGTP";
}

int ROIBasedGTP::expectedDependencies() const
{
  return 2;
}

void ROIBasedGTP::setParameters()
{
  GroundTruthProvider::setParameters();
  nbGeneratedPatches = ParamInt(5, 0, 20);
  maxPosNoise = ParamInt(5, 0, 20);
  maxRatioNoise = ParamFloat(0.1, 0.0, 1.0);

  params()->define<ParamInt>("nbGeneratedPatches", &nbGeneratedPatches);
  params()->define<ParamInt>("maxPosNoise", &maxPosNoise);
  params()->define<ParamFloat>("maxRatioNoise", &maxRatioNoise);
}

std::vector<cv::Rect_<float>> ROIBasedGTP::generateROIs()
{
  std::uniform_int_distribution<int> pos_distribution(-maxPosNoise, maxPosNoise);
  // See formula below to understand how negative ratios are used, just making sure that center is no transformation
  std::uniform_real_distribution<double> ratio_distribution(-maxRatioNoise, maxRatioNoise);

  int width = this->getSourceImg().cols;
  int height = this->getSourceImg().rows;
  cv::Size img_size(width, height);

  std::vector<cv::Rect_<float>> rois;
  for (const auto& entry : getDependency(1).getRois())
  {
    cv::Rect_<float> original_rect = Utils::toRect(entry.second);
    rois.push_back(original_rect);

    for (int i = 0; i < nbGeneratedPatches; i++)
    {
      // Note: no distinction between width and height because roi are squares
      double original_size = original_rect.width;
      double size_ratio = ratio_distribution(engine);
      double size_increase;
      if (size_ratio > 0)
      {
        size_increase = original_size * size_ratio;
      }
      else
      {
        size_increase = -original_size / size_ratio;
      }
      //
      cv::Point2f new_tl = original_rect.tl();
      new_tl.x += pos_distribution(engine) - size_increase / 2;
      new_tl.y += pos_distribution(engine) - size_increase / 2;

      int new_size = original_size + size_increase;
      cv::Rect2f new_roi(new_tl, cv::Size(new_size, new_size));

      if (!Utils::isContained(new_roi, img_size))
      {
        continue;
      }
      rois.push_back(new_roi);
    }
  }
  return rois;
}

const cv::Mat& ROIBasedGTP::getSourceImg()
{
  return *(getDependency(0).getImg());
}

}  // namespace Filters
}  // namespace Vision
