#include <opencv2/core/core.hpp>
#include "Filters/Basics/Invert.hpp"

namespace Vision
{
namespace Filters
{
void Invert::process()
{
  cv::Mat src = *(getDependency().getImg());
  // Other option : cv::bitwise_not (src, src);
  img() = 255 - src;
}
}  // namespace Filters
}  // namespace Vision
