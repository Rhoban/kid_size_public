#include "FeaturesFactory.hpp"

#include "TagsDetector.hpp"

#include "../FilterFactory.hpp"

namespace Vision
{
namespace Filters
{
void registerFeaturesFilters(FilterFactory* ff)
{
  ff->registerBuilder("TagsDetector", []() { return std::unique_ptr<Filter>(new TagsDetector); });
}

}  // namespace Filters
}  // namespace Vision
