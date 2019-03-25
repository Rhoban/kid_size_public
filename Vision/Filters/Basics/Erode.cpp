#include "Filters/Basics/Erode.hpp"

#include <opencv2/imgproc/imgproc.hpp>

namespace Vision
{
namespace Filters
{
void Erode::setParameters()
{
  shape = ParamInt(2, 0, 2);
  kWidth = ParamInt(5, 1, 11);
  kHeight = ParamInt(5, 1, 11);
  params()->define<ParamInt>("Shape", &shape);
  params()->define<ParamInt>("kernel width", &kWidth);
  params()->define<ParamInt>("kernel height", &kHeight);
}

void Erode::process()
{
  cv::Mat src = *(getDependency().getImg());
  cv::Mat kernel = getStructuringElement(shape, cv::Size(kWidth, kHeight));
  erode(*(getDependency().getImg()), img(), kernel);
}
}  // namespace Filters
}  // namespace Vision
