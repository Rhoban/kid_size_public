#pragma once

#include "Filters/Filter.hpp"

namespace Vision
{
namespace Filters
{
/**
 * MaskOr
 * Does a logic OR between the inputs
 */
class MaskOr : public Filter
{
public:
  MaskOr() : Filter("MaskOr")
  {
  }

  virtual std::string getClassName() const override
  {
    return "MaskOr";
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
