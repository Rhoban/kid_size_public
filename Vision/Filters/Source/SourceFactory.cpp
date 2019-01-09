#include "SourceFactory.hpp"

#include "SourceLogs.hpp"
#include "SourceOpenCV.hpp"
#include "SourceVideoProtobuf.hpp"

#include "../FilterFactory.hpp"

#ifdef KID_SIZE_USES_FLYCAPTURE
  #include "SourcePtGrey.hpp"
#endif

namespace Vision {
namespace Filters {

void registerSourceFilters(FilterFactory * ff) {
  ff->registerBuilder("SourceLogs"  , [](){return std::unique_ptr<Filter>(new SourceLogs  );});
  ff->registerBuilder("SourceOpenCV", [](){return std::unique_ptr<Filter>(new SourceOpenCV);});
  ff->registerBuilder("SourceVideoProtobuf",
                      [](){return std::unique_ptr<Filter>(new SourceVideoProtobuf);});
#ifdef KID_SIZE_USES_FLYCAPTURE
  ff->registerBuilder("SourcePtGrey", [](){return std::unique_ptr<Filter>(new SourcePtGrey);});
#endif
}

}
}
