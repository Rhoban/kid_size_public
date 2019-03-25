#pragma once

#include "Filters/Filter.hpp"

namespace Vision
{
namespace Filters
{
/**
 * Erode
 *
 * Erode the image with a kernel of a given width, height and shape
 */
class Erode : public Filter
{
public:
  Erode() : Filter("Erode")
  {
  }

  virtual std::string getClassName() const override
  {
    return "Erode";
  }

protected:
  /**
   * @Inherit
   */
  virtual void process() override;

  virtual void setParameters() override;

private:
  /**
   * Shape :
   * 0 - Rect
   * 1 - Cross
   * 2 - Ellipse
   */
  ParamInt shape;
  ParamInt kWidth;
  ParamInt kHeight;
};
}  // namespace Filters
}  // namespace Vision
