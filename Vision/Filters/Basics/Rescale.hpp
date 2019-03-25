#pragma once

#include "Filters/Filter.hpp"

namespace Vision
{
namespace Filters
{
/**
 * Rescale
 *
 * Divide the width and the height of the given image by a ratio
 */
class Rescale : public Filter
{
public:
  Rescale() : Filter("Rescale")
  {
  }

  virtual std::string getClassName() const override
  {
    return "Rescale";
  }

protected:
  /**
   * @Inherit
   */
  virtual void process() override;

  virtual void setParameters() override;

private:
  ParamFloat _ratio;
};
}  // namespace Filters
}  // namespace Vision
