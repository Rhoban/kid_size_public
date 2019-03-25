#include <opencv2/core/core.hpp>
#include "Filters/Basics/Mult.hpp"

namespace Vision
{
namespace Filters
{
void Mult::setParameters()
{
  _ratio = ParamFloat(10, 0, 255);
  params()->define<ParamFloat>("ratio", &_ratio);
}

void Mult::process()
{
  img() = *(getDependency().getImg()) * _ratio;
}
}  // namespace Filters
}  // namespace Vision
