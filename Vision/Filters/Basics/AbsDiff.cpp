#include <opencv2/core/core.hpp>
#include "Filters/Basics/AbsDiff.hpp"
#include <string>
#include <cstdlib>

namespace Vision
{
namespace Filters
{
void AbsDiff::process()
{
  cv::Mat A = *(getDependency(_dependencies[0]).getImg());
  cv::Mat B = *(getDependency(_dependencies[1]).getImg());
  img() = abs(A - B);
}
}  // namespace Filters
}  // namespace Vision
