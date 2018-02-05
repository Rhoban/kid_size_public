#include "ColorsFactory.hpp"

#include "ChannelSelector.hpp"
#include "ColorBounding.hpp"
#include "ColorConverter.hpp"

#include "../FilterFactory.hpp"

namespace Vision {
namespace Filters {

void registerColorsFilters(FilterFactory * ff) {
  ff->registerBuilder("ChannelSelector", [](){return std::unique_ptr<Filter>(new ChannelSelector());});
  ff->registerBuilder("ColorBounding"  , [](){return std::unique_ptr<Filter>(new ColorBounding()  );});
  ff->registerBuilder("ColorConverter" , [](){return std::unique_ptr<Filter>(new ColorConverter() );});
}

}
}
