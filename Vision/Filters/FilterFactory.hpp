#pragma once

#include "rhoban_utils/serialization/factory.h"

#include "Filters/Filter.hpp"

namespace Vision
{
namespace Filters
{
class FilterFactory : public rhoban_utils::Factory<Filter>
{
public:
  FilterFactory();
};
}  // namespace Filters
}  // namespace Vision
