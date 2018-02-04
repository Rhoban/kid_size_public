#include "PatchFactory.hpp"

#include "RoiToPatches.hpp"
#include "PatchRecorder.hpp"

#include "../FilterFactory.hpp"

namespace Vision {
namespace Filters {
void registerPatchFilters() {
  FilterFactory::registerClass<RoiToPatches>("RoiToPatches");
  FilterFactory::registerClass<PatchRecorder>("PatchRecorder");
}
}
}
