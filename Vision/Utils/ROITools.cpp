#include "ROITools.hpp"

#include "RotatedRectUtils.hpp"

namespace Vision
{
namespace Utils
{
cv::Rect resizeROI(const cv::Rect& roi, const cv::Mat& srcImg, const cv::Mat& dstImg)
{
  float ratio = ((float)dstImg.cols) / srcImg.cols;
  cv::Rect newROI = cv::Rect(roi);
  newROI.x = newROI.x * ratio;
  newROI.y = newROI.y * ratio;
  newROI.width = newROI.width * ratio;
  newROI.height = newROI.height * ratio;

  newROI = cropRect(newROI, dstImg);

  return newROI;
}

bool isContained(const cv::Rect& rect, const cv::Size& size)
{
  const cv::Point2f& tl = rect.tl();
  const cv::Point2f& br = rect.br();
  return (tl.x >= 0 && br.x <= size.width - 1 && tl.y >= 0 && br.y <= size.height - 1);
}

cv::Rect cropRect(const cv::Rect& roi, const cv::Mat& img)
{
  return cropRect(roi, img.size());
}

cv::Rect cropRect(const cv::Rect& roi, const cv::Size& size)
{
  cv::Rect newROI = cv::Rect(roi);
  // Cropping ROI
  if (newROI.x < 0)
  {
    newROI.width += newROI.x;
    newROI.x = 0;
  }
  if (newROI.y < 0)
  {
    newROI.height += newROI.y;
    newROI.y = 0;
  }
  int maxHeight = size.height - newROI.y;
  int maxWidth = size.width - newROI.x;
  if (newROI.width >= maxWidth)
    newROI.width = maxWidth;
  if (newROI.height >= maxHeight)
    newROI.height = maxHeight;

  return newROI;
}

cv::RotatedRect resizeROI(const cv::RotatedRect& roi, const cv::Mat& roiImg, const cv::Mat& dstImg)
{
  float ratio = ((float)dstImg.cols) / roiImg.cols;

  cv::Point center(roi.center.x * ratio, roi.center.y * ratio);
  cv::Size size(roi.size.width * ratio, roi.size.height * ratio);

  cv::RotatedRect newROI(center, size, roi.angle);

  return newROI;
}

cv::Rect toRect(const cv::RotatedRect& rotatedRect)
{
  // TODO: Ask Remi if there is a specific reason to test this case
  if (rotatedRect.angle == 0)
  {
    cv::Rect rect(rotatedRect.center.x - rotatedRect.size.width / 2, rotatedRect.center.y - rotatedRect.size.height / 2,
                  rotatedRect.size.width, rotatedRect.size.height);
    return rect;
  }
  else
  {
    return rotatedRect.boundingRect();
  }
}

cv::RotatedRect toRotatedRect(const cv::Rect& rect)
{
  cv::Point sum = rect.tl() + rect.br();
  cv::Point2f center = cv::Point2f(sum.x / 2.0, sum.y / 2.0);
  return cv::RotatedRect(center, rect.size(), 0);
}

std::pair<cv::Rect, cv::Mat> getROIMask(const cv::RotatedRect& roi, const cv::Mat& srcImg)
{
  // Get the bounding box of the provided ROI
  cv::Rect boundingRect = cropRect(toRect(roi), srcImg);
  // Throwing errors in degenerated cases
  if (boundingRect.width < 2 || boundingRect.height < 2)
  {
    throw std::runtime_error("getROIMask: width or height of the boundingRect is smaller than 1");
  }
  // Create a mask of boundingRect size
  cv::Mat rotatedRoiMask(cv::Size(boundingRect.width, boundingRect.height), 0, cv::Scalar(0));
  // rectForMask has the same size and angle than roi, but a different center
  // (the initial roi applies on the fll scale image)
  cv::RotatedRect rectForMask(cv::Point(boundingRect.width / 2, boundingRect.height / 2), roi.size, roi.angle);
  // Draw the rotated rectangle inside the mask
  drawRotatedRectangle(rotatedRoiMask, rectForMask, cv::Scalar(255), -1);

  return { boundingRect, rotatedRoiMask };
}

bool isOverlapping(const cv::Rect& r1, const cv::Rect& r2)
{
  return (r1 & r2).area() > 0;
}

double computeOverlapRatio(const cv::Rect& r1, const cv::Rect& r2)
{
  return ((double)(r1 & r2).area()) / std::min(r1.area(), r2.area());
}

}  // namespace Utils
}  // namespace Vision
