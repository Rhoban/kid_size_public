#include "PatchRecorder.hpp"

#include "PatchProvider.hpp"

#include "rhoban_utils/logging/logger.h"
#include "rhoban_utils/timing/benchmark.h"

#include <opencv2/opencv.hpp>

using rhoban_utils::Benchmark;

rhoban_utils::Logger logger("PatchRecorder");

namespace Vision
{
namespace Filters
{
PatchRecorder::PatchRecorder() : Filter("PatchRecorder"), imgId(0), prefix("image"), curr_period(0)
{
}

std::string PatchRecorder::getClassName() const
{
  return "PatchRecorder";
}

void PatchRecorder::setParameters()
{
  period = ParamInt(1, -1, 100);

  params()->define<ParamInt>("period", &period);
}

Json::Value PatchRecorder::toJson() const
{
  Json::Value v = Filter::toJson();
  v["prefix"] = prefix;
  return v;
}
void PatchRecorder::fromJson(const Json::Value& v, const std::string& dir_name)
{
  Filter::fromJson(v, dir_name);
  rhoban_utils::tryRead(v, "prefix", &prefix);
}

int PatchRecorder::expectedDependencies() const
{
  return 1;
}

void PatchRecorder::process()
{
  const PatchProvider& patch_provider = dynamic_cast<const PatchProvider&>(getDependency());

  // If period is disabled, skip execution
  if (period < 1)
    return;

  // If period has not ended, skip
  curr_period++;
  if (curr_period < period)
    return;

  // Reset period and write patches
  curr_period = 0;

  Benchmark::open("Writing patches");
  const std::vector<cv::Mat>& patches = patch_provider.getPatches();
  for (int patchId = 0; patchId < (int)patches.size(); patchId++)
  {
    char suffix[50];
    sprintf(suffix, "%06d_patch_%02d.png", imgId, patchId);
    std::string filename = prefix + suffix;
    if (!cv::imwrite(filename.c_str(), patches[patchId]))
    {
      logger.warning("Failed imwrite to '%s'", filename.c_str());
    }
  }
  Benchmark::close("Writing patches");
  imgId++;
}

}  // namespace Filters
}  // namespace Vision
