#pragma once

#include "Filters/Filter.hpp"

namespace Vision
{
namespace Filters
{
/**
 * Invert
 *
 * Invert a CV_8U image (dst[i,j] = 255 - src[i,j])
 */
class Invert : public Filter
{
public:
  Invert() : Filter("Invert")
  {
  }

  virtual std::string getClassName() const override
  {
    return "Invert";
  }

protected:
  /**
   * @Inherit
   */
  virtual void process() override;
};
}  // namespace Filters
}  // namespace Vision
