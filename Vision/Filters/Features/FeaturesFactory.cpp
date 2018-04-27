#include "FeaturesFactory.hpp"

#include "VisualCompass.hpp"
#include "TagsDetector.hpp"

#include "../FilterFactory.hpp"

namespace Vision {
namespace Filters {
void registerFeaturesFilters(FilterFactory * ff)
{
  ff->registerBuilder("VisualCompass", [](){return std::unique_ptr<Filter>(new VisualCompass);});
  ff->registerBuilder("TagsDetector" , [](){return std::unique_ptr<Filter>(new TagsDetector );});
}

}
}
