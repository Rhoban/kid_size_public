#include "ColorsFactory.hpp"

#include "ChannelSelector.hpp"
#include "ColorBounding.hpp"
#include "ColorConverter.hpp"

#include "../FilterFactory.hpp"

namespace Vision {
namespace Filters {
void registerColorsFilters() {
  FilterFactory::registerClass<ChannelSelector>("ChannelSelector");
  FilterFactory::registerClass<ColorBounding>("ColorBounding");
  FilterFactory::registerClass<ColorConverter>("ColorConverter");
}
}
}
