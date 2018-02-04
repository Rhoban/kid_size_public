#include <opencv2/core/core.hpp>
#include "Filters/Basics/Diff.hpp"

#include <iostream>

namespace Vision {
namespace Filters {

Diff::Diff(const std::string &name, const std::string &A, const std::string &B,
           Frequency::type frequency)
    : Filter(name, {A, B}, frequency) {}

void Diff::process() {
  std::string aName = _dependencies[0];
  std::string bName = _dependencies[1];
  cv::Mat A = *(getDependency(aName).getImg());
  cv::Mat B = *(getDependency(bName).getImg());
  img() = A - B;
}
}
}
