#ifndef VISION_UTILS_SEGMENT_HPP
#define VISION_UTILS_SEGMENT_HPP

#include <opencv2/core/core.hpp>

namespace Vision {
namespace Utils {

class Segment : public std::pair<cv::Point, cv::Point> {
public:
  Segment() : std::pair<cv::Point, cv::Point>() {}

  Segment(cv::Point p1, cv::Point p2)
      : std::pair<cv::Point, cv::Point>(p1, p2) {}

  cv::Point center() const {
    return cv::Point((first.x + second.x) / 2, (first.y + second.y) / 2);
  }
};
}
}

#endif // VISION_UTILS_SEGMENT_HPP
