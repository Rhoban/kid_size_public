#include "SourceFactory.hpp"

#include "SourceLogs.hpp"
#include "SourcePtGrey.hpp"

#include "../FilterFactory.hpp"

namespace Vision {
namespace Filters {
void registerSourceFilters() {
  FilterFactory::registerClass<SourceLogs>("SourceLogs");
  FilterFactory::registerClass<SourcePtGrey>("SourcePtGrey");
}
}
}
