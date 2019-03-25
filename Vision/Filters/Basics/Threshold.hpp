#pragma once

#include "Filters/Filter.hpp"

namespace Vision
{
namespace Filters
{
/**
 * Threshold
 *
 * Apply a threshold on a given image, different types of threshold might be
 * applied
 * WARNING: This filter is not adapted for working with floating images
 */
class Threshold : public Filter
{
public:
  Threshold() : Filter("Threshold")
  {
  }

  virtual std::string getClassName() const override
  {
    return "Threshold";
  }

protected:
  /**
   * @Inherit
   */
  virtual void process() override;

  virtual void setParameters() override;

private:
  ParamInt threshold;
  /**
   * Threshold Type:
   * 0: Binary
   * 1: Binary Inverted
   * 2: Truncate
   * 3: To Zero
   * 4: To Zero Inverted
   */
  ParamInt thresholdType;
};
}  // namespace Filters
}  // namespace Vision
