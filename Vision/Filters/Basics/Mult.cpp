#include <opencv2/core/core.hpp>
#include "Filters/Basics/Mult.hpp"

namespace Vision {
namespace Filters {

Mult::Mult(const std::string &name, const std::string &source,
           Frequency::type frequency)
    : Filter(name, {source}, frequency) {}

void Mult::setParameters() {
  _ratio = ParamFloat(10, 0, 255);
  params()->define<ParamFloat>("ratio", &_ratio);
}

void Mult::process() { img() = *(getDependency().getImg()) * _ratio; }
}
}
