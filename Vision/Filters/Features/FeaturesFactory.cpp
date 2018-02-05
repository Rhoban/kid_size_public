#include "FeaturesFactory.hpp"

#include "FREAK.hpp"
#include "VisualCompass.hpp"
#include "TagsDetector.hpp"

#include "../FilterFactory.hpp"

namespace Vision {
namespace Filters {
void registerFeaturesFilters(FilterFactory * ff)
{
  ff->registerBuilder("FREAK"        , [](){return std::unique_ptr<Filter>(new FREAK        );});
  ff->registerBuilder("VisualCompass", [](){return std::unique_ptr<Filter>(new VisualCompass);});
  ff->registerBuilder("TagsDetector" , [](){return std::unique_ptr<Filter>(new TagsDetector );});
}

}
}
