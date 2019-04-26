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

  std::random_device rd;     // only used once to initialise (seed) engine
  std::mt19937 rng(rd());    // random-number engine used (Mersenne-Twister in this case)
  std::uniform_int_distribution<int> uni(-5, 5); // guaranteed unbiased
  
  int width = this->getSourceImg().cols;
  int height = this->getSourceImg().rows;

  std::vector<cv::Rect_<float>> rois;
  for (const auto& entry: getDependency(1).getRois())
  {
    rois.push_back(Utils::toRect(entry.second));
    auto rect = Utils::toRect(entry.second);
    // logger.log("%d, %d, %d, %d", rect.x, rect.y, rect.width, rect.height);

    // Data Augmentation

    int x_offset;
    int y_offset;
    int nbTranslations = 5;
    for(int i = 0 ; i < nbTranslations ; i++)
    {
      x_offset = uni(rng);
      y_offset = uni(rng);

      while(rect.x+x_offset < 0 || rect.x+x_offset+rect.width > width-1 ||
	    rect.y+y_offset < 0 || rect.y+y_offset+rect.height > height-1)
      {
	x_offset = uni(rng);
	y_offset = uni(rng);
      }

      rect.x += x_offset;
      rect.y += y_offset;
      
      rois.push_back(rect);      
      // logger.log("%d, %d", x_offset, y_offset);
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
