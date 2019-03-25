#include "Utils/PatchTools.hpp"

#include "Utils/ROITools.hpp"

namespace Vision
{
namespace Utils
{
// Specificity of integral images: one more column and row than original image
cv::Rect iiCropRect(const cv::Rect& patch, const cv::Size& iiSize)
{
  return Utils::cropRect(patch, cv::Size(iiSize.width - 1, iiSize.height - 1));
}

double getPatchSum(const cv::Rect& patch, const cv::Mat& integralImage, bool crop)
{
  cv::Rect rect = patch;
  if (crop)
  {
    rect = iiCropRect(patch, integralImage.size());
  }

  cv::Point2i tl, br;
  tl = rect.tl();
  br = rect.br();

  int A, B, C, D;
  A = integralImage.at<int>(tl.y, tl.x);
  B = integralImage.at<int>(tl.y, br.x);
  C = integralImage.at<int>(br.y, tl.x);
  D = integralImage.at<int>(br.y, br.x);

  return (A + D) - (B + C);
}

double getPatchDensity(const cv::Rect& patch, const cv::Mat& integralImage, bool crop)
{
  cv::Rect rect = patch;
  if (crop)
  {
    rect = iiCropRect(patch, integralImage.size());
  }

  if (rect.area() == 0)
  {
    throw std::runtime_error("Patch is out of image");
  }

  return getPatchSum(rect, integralImage, false) / rect.area();
}

}  // namespace Utils
}  // namespace Vision
