#pragma once

#include "Filters/Filter.hpp"

namespace Vision
{
namespace Filters
{
/**
 * Blur
 *
 * Blur the image with a kernel of a given width and height
 */
class Blur : public Filter
{
public:
  Blur() : Filter("Blur")
  {
  }

  virtual std::string getClassName() const override
  {
    return "Blur";
  }

protected:
  /**
   * @Inherit
   */
  virtual void process() override;

  virtual void setParameters() override;

private:
  ParamInt kWidth;
  ParamInt kHeight;
};
}  // namespace Filters
}  // namespace Vision
