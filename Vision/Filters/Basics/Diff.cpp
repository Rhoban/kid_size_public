#include <opencv2/core/core.hpp>
#include "Filters/Basics/Diff.hpp"

#include <iostream>

namespace Vision
{
namespace Filters
{
void Diff::process()
{
  std::string aName = _dependencies[0];
  std::string bName = _dependencies[1];
  cv::Mat A = *(getDependency(aName).getImg());
  cv::Mat B = *(getDependency(bName).getImg());
  img() = A - B;
}
}  // namespace Filters
}  // namespace Vision
