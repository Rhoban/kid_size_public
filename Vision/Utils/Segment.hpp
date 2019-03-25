#ifndef VISION_UTILS_SEGMENT_HPP
#define VISION_UTILS_SEGMENT_HPP

#include <opencv2/core/core.hpp>

namespace Vision
{
namespace Utils
{
class Segment : public std::pair<cv::Point2f, cv::Point2f>
{
public:
  Segment() : std::pair<cv::Point2f, cv::Point2f>()
  {
  }

  Segment(cv::Point2f p1, cv::Point2f p2) : std::pair<cv::Point2f, cv::Point2f>(p1, p2)
  {
  }

  cv::Point2f center() const
  {
    return cv::Point2f((first.x + second.x) / 2, (first.y + second.y) / 2);
  }
};
}  // namespace Utils
}  // namespace Vision

#endif  // VISION_UTILS_SEGMENT_HPP
