#pragma once

#include <opencv2/core/core.hpp>

namespace Vision
{
namespace Utils
{
// Resize a roi in srcImg to the corresponding in dstImg
cv::Rect resizeROI(const cv::Rect& roi, const cv::Mat& srcImg, const cv::Mat& dstImg);

// Test if 'rect' is Contained in size
bool isContained(const cv::Rect& rect, const cv::Size& size);

// Ensure that the rect is entirely inside the given img
cv::Rect cropRect(const cv::Rect& roi, const cv::Mat& img);
cv::Rect cropRect(const cv::Rect& roi, const cv::Size& size);

// Resize a roi in srcImg to the corresponding in dstImg
cv::RotatedRect resizeROI(const cv::RotatedRect& roi, const cv::Mat& roiImg, const cv::Mat& dstImg);

/// Return the smallest englobing rectangle
cv::Rect toRect(const cv::RotatedRect& rotatedRect);

/// To avoid compatibility issues
cv::RotatedRect toRotatedRect(const cv::Rect& rect);

/// Return:
/// first: the non-rotated rect which contains the image associated
/// second: A Mask to be used on the image
/// Throws a runtime_error if boundingRect of Roi in the image would be too thin
std::pair<cv::Rect, cv::Mat> getROIMask(const cv::RotatedRect& roi, const cv::Mat& srcImg);

bool isOverlapping(const cv::Rect& r1, const cv::Rect& r2);
/**
 * Get area of intersection of both rectangles divided by the minimum area between r1 and r2
 * output is in [0,1]
 */
double computeOverlapRatio(const cv::Rect& r1, const cv::Rect& r2);

}  // namespace Utils
}  // namespace Vision
