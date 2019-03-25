#include "Filters/Basics/Dilate.hpp"

#include <opencv2/imgproc/imgproc.hpp>

namespace Vision
{
namespace Filters
{
void Dilate::setParameters()
{
  shape = ParamInt(2, 0, 2);
  kWidth = ParamInt(5, 1, 11);
  kHeight = ParamInt(5, 1, 11);
  params()->define<ParamInt>("Shape", &shape);
  params()->define<ParamInt>("kernel width", &kWidth);
  params()->define<ParamInt>("kernel height", &kHeight);
}

void Dilate::process()
{
  cv::Mat src = *(getDependency().getImg());
  cv::Mat kernel = getStructuringElement(shape, cv::Size(kWidth, kHeight));
  dilate(*(getDependency().getImg()), img(), kernel);
}
}  // namespace Filters
}  // namespace Vision
