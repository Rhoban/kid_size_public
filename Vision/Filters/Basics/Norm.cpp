#include <opencv2/core/core.hpp>
#include "Filters/Basics/Norm.hpp"

namespace Vision
{
namespace Filters
{
void Norm::process()
{
  cv::Mat src = *(getDependency().getImg());
  cv::Mat dst = cv::Mat(src.rows, src.cols, CV_8U);
  for (int row = 0; row < src.rows; row++)
  {
    for (int col = 0; col < src.cols; col++)
    {
      cv::Vec3b val = src.at<cv::Vec3b>(row, col);
      int norm = sqrt(val[0] * val[0] + val[1] * val[1] + val[2] * val[2]);
      unsigned char newVal = 255;
      if (norm < 255)
        newVal = norm;
      dst.at<unsigned char>(row, col) = newVal;
    }
  }
  img() = dst;
}
}  // namespace Filters
}  // namespace Vision
