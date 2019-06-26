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

std::vector<cv::Rect_<float>> ROIBasedGTP::generateROIs()
{
  int width = this->getSourceImg().cols;
  int height = this->getSourceImg().rows;
  cv::Size img_size(width, height);

  std::vector<cv::Rect_<float>> rois;
  for (const auto& entry : getDependency(1).getRois())
  {
    rois.push_back(Utils::toRect(entry.second));
  }
  return rois;
}

const cv::Mat& ROIBasedGTP::getSourceImg()
{
  return *(getDependency(0).getImg());
}

}  // namespace Filters
}  // namespace Vision
