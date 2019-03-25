#pragma once

#include "Filters/Filter.hpp"

namespace Vision
{
namespace Filters
{
/**
 * Mult
 *
 * Multiply each channel of the image by the given ratio
 */
class Mult : public Filter
{
public:
  Mult() : Filter("Mult")
  {
  }

  virtual std::string getClassName() const override
  {
    return "Mult";
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
