#include <opencv2/core/core.hpp>
#include "Filters/Basics/Mask.hpp"

namespace Vision
{
namespace Filters
{
void Mask::process()
{
  std::string srcName = _dependencies[0];
  std::string maskName = _dependencies[1];

  cv::Mat src = *(getDependency(srcName).getImg());
  cv::Mat mask = *(getDependency(maskName).getImg());

  cv::Mat tmp = src.clone();
  float ratio = ((float)src.cols) / mask.cols;

  // TODO the map seems to be parsed in the wrong order, I think it should be
  // rows then cols.
  // I did a quick test but for some reason the results change significantly on
  // the same image with
  // the same code. To be tested seriously, no time now.
  for (int col = 0; col < tmp.cols; col++)
  {
    for (int row = 0; row < tmp.rows; row++)
    {
      if (mask.at<char>(row / ratio, col / ratio) == 0)
      {
        if (src.channels() == 3)
        {
          tmp.at<cv::Vec3b>(row, col) = cv::Vec3b(0, 0, 0);
        }
        else if (src.channels() == 1)
        {
          tmp.at<char>(row, col) = 0;
        }
      }
    }
  }
  img() = tmp;
}
}  // namespace Filters
}  // namespace Vision
