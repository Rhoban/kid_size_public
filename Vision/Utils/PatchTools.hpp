#pragma once

#include <opencv2/core/core.hpp>

/// A list of common function useful for working on patches on integral images

namespace Vision
{
namespace Utils
{
/// Return the sum of pixel values over all the pixels inside the patch
/// If crop is activated, then the patch is first cropped to be inside the image
/// If patch area is 0, return 0
double getPatchSum(const cv::Rect& patch, const cv::Mat& integralImage, bool crop = true);

/// Returns the average pixel value inside the patch
/// If crop is activated, then the patch is first cropped to be inside the image
/// If patch area is 0, throws a runtime_error
double getPatchDensity(const cv::Rect& patch, const cv::Mat& integralImage, bool crop = true);

}  // namespace Utils
}  // namespace Vision
