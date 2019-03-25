#pragma once

#include "Filters/Filter.hpp"

namespace Vision
{
namespace Filters
{
/**
 * Dilate
 *
 * Dilate the image with a kernel of a given width, height and shape
 */
class Dilate : public Filter
{
public:
  Dilate() : Filter("Dilate")
  {
  }

  virtual std::string getClassName() const override
  {
    return "Dilate";
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
