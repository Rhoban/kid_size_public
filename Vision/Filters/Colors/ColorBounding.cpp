#include "Filters/Colors/ColorBounding.hpp"

#include <opencv2/imgproc/imgproc.hpp>

namespace Vision
{
namespace Filters
{
void ColorBounding::setParameters()
{
  minY = ParamInt(124, 0, 255, ParameterType::PARAM);
  maxY = ParamInt(124, 0, 255, ParameterType::PARAM);
  minU = ParamInt(124, 0, 255, ParameterType::PARAM);
  maxU = ParamInt(124, 0, 255, ParameterType::PARAM);
  minV = ParamInt(124, 0, 255, ParameterType::PARAM);
  maxV = ParamInt(124, 0, 255, ParameterType::PARAM);

  params()->define<ParamInt>("minY", &minY);
  params()->define<ParamInt>("maxY", &maxY);
  params()->define<ParamInt>("minU", &minU);
  params()->define<ParamInt>("maxU", &maxU);
  params()->define<ParamInt>("minV", &minV);
  params()->define<ParamInt>("maxV", &maxV);
}

void ColorBounding::process()
{
  cv::Mat src = *(getDependency().getImg());

  cv::Mat temp(src.size(), 0, cv::Scalar(0));  // Unsigned char mat
  cv::Scalar lowLimit = cv::Scalar(minY, minU, minV);
  cv::Scalar highLimit = cv::Scalar(maxY, maxU, maxV);

  inRange(src, lowLimit, highLimit, temp);

  img() = temp;
}
}  // namespace Filters
}  // namespace Vision
