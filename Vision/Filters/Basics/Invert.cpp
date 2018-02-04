#include <opencv2/core/core.hpp>
#include "Filters/Basics/Invert.hpp"

namespace Vision {
namespace Filters {

Invert::Invert(const std::string &name, const std::string &source,
               Frequency::type frequency)
    : Filter(name, {source}, frequency) {}

void Invert::process() {
  cv::Mat src = *(getDependency().getImg());
  // Other option : cv::bitwise_not (src, src);
  img() = 255 - src;
}
}
}
