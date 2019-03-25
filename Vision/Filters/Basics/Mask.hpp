#pragma once

#include "Filters/Filter.hpp"

namespace Vision
{
namespace Filters
{
/**
 * Mask
 *
 * Apply a mask on an image (if mask[i,j] = 0 then img[i,j] = 0)
 * mask has to be an image of the same width/height ratio as src
 */
class Mask : public Filter
{
public:
  Mask() : Filter("Mask")
  {
  }

  virtual std::string getClassName() const override
  {
    return "Mask";
  }
  virtual int expectedDependencies() const override
  {
    return 2;
  }

protected:
  /**
   * @Inherit
   */
  virtual void process() override;
};
}  // namespace Filters
}  // namespace Vision
