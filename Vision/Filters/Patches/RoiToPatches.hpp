#pragma once

#include "PatchProvider.hpp"

namespace Vision
{
namespace Filters
{
/// This filter allows to extract patches in an image from ROI
///
/// Takes two dependencies:
/// 1. Filter providing the image from which patch will be extracted
/// 2. Filter providing the regions of interest
class RoiToPatches : public PatchProvider
{
public:
  RoiToPatches();

  virtual std::string getClassName() const override;
  virtual int expectedDependencies() const;

  virtual void setParameters() override;

protected:
  virtual void process() override;

  /// Define the tagging of images on the output image
  /// 0 -> No tagging
  /// 1 -> Draw red rectangles around Patches
  ParamInt tagLevel;
};

}  // namespace Filters
}  // namespace Vision
