#include "Filters/Basics/Threshold.hpp"

#include <opencv2/imgproc/imgproc.hpp>

namespace Vision
{
namespace Filters
{
void Threshold::setParameters()
{
  threshold = ParamInt(10, 0, 255);
  thresholdType = ParamInt(0, 0, 4);
  params()->define<ParamInt>("threshold", &threshold);
  params()->define<ParamInt>("thresholdType", &thresholdType);
}

void Threshold::process()
{
  cv::threshold(*(getDependency().getImg()), img(), threshold, 255, thresholdType);
}
}  // namespace Filters
}  // namespace Vision
