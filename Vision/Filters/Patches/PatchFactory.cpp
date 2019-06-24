#include "PatchFactory.hpp"

#include "ROIRandomizer.hpp"
#include "RoiToPatches.hpp"
#include "PatchRecorder.hpp"

#include "../FilterFactory.hpp"

namespace Vision
{
namespace Filters
{
void registerPatchFilters(FilterFactory* ff)
{
  ff->registerBuilder("ROIRandomizer", []() { return std::unique_ptr<Filter>(new ROIRandomizer); });
  ff->registerBuilder("RoiToPatches", []() { return std::unique_ptr<Filter>(new RoiToPatches); });
  ff->registerBuilder("PatchRecorder", []() { return std::unique_ptr<Filter>(new PatchRecorder); });
}

}  // namespace Filters
}  // namespace Vision
