#include "SourceFactory.hpp"

#include "SourceLogs.hpp"

#include "../FilterFactory.hpp"

#if KID_SIZE_USES_FLYCAPTURE
  #include "SourcePtGrey.hpp"
#endif

namespace Vision {
namespace Filters {

void registerSourceFilters(FilterFactory * ff) {
  ff->registerBuilder("SourceLogs"  , [](){return std::unique_ptr<Filter>(new SourceLogs  );});
#if KID_SIZE_USES_FLYCAPTURE
  ff->registerBuilder("SourcePtGrey", [](){return std::unique_ptr<Filter>(new SourcePtGrey);});
#endif
}

}
}
