#include "FeaturesFactory.hpp"

#include "FREAK.hpp"
#include "VisualCompass.hpp"
#include "TagsDetector.hpp"

#include "../FilterFactory.hpp"

namespace Vision {
namespace Filters {
void registerFeaturesFilters()
{
    FilterFactory::registerClass<FREAK>("FREAK");
    FilterFactory::registerClass<VisualCompass>("VisualCompass");
    FilterFactory::registerClass<TagsDetector>("TagsDetector");
}

}
}
