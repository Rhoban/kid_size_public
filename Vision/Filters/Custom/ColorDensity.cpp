#include "Filters/Custom/ColorDensity.hpp"

using namespace std;

namespace Vision
{
namespace Filters
{
ColorDensity::ColorDensity() : Filter("ColorDensity")
{
}

ColorDensity::~ColorDensity()
{
}

void ColorDensity::setParameters()
{
  /* define the size of the squared kernel side in pixel */
  kernel_size = ParamInt(30, 2, 100);
  params()->define<ParamInt>("kernel_size", &kernel_size);
  resize_factor = ParamFloat(1.0, 0.0, 1.0);
  params()->define<ParamFloat>("resize_factor", &resize_factor);
}

void ColorDensity::process()
{
  std::string greenIIName = _dependencies[0];  // Integral image
  cv::Mat greenII = (getDependency(greenIIName).getImg())->clone();
  int col_nb = (int)(resize_factor * (greenII.cols - 1));
  int row_nb = (int)(resize_factor * (greenII.rows - 1));
  img() = cv::Mat(row_nb, col_nb, 0);

  for (int x = 0; x < col_nb; x++)
  {
    for (int y = 0; y < row_nb; y++)
    {
      int s = kernel_size / 2;
      int xm = max((int)(x / resize_factor) - s, 0), ym = max((int)(y / resize_factor) - s, 0);
      int xM = min((int)(x / resize_factor) + s, greenII.cols - 1),
          yM = min((int)(y / resize_factor) + s, greenII.rows - 1);
      int kernel_vol = abs((xM - xm) * (yM - ym));
      // the initial green input is binary but its values are either 0 or 255 (and not 0 or 1)
      // That's why the density calculation is in the range [0, 255]
      double sum_sq =
          greenII.at<int>(yM, xM) - greenII.at<int>(ym, xM) - greenII.at<int>(yM, xm) + greenII.at<int>(ym, xm);
      int mean_v = (int)(sum_sq / kernel_vol);
      img().at<uchar>(y, x) = min(mean_v, 255);
    }
  }
}

}  // namespace Filters
}  // namespace Vision
