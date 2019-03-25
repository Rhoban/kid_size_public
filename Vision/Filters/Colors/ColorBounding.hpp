#pragma once

#include <vector>
#include "Filters/Filter.hpp"

namespace Vision
{
namespace Filters
{
/**
 * ColorBounding
 *
 * Detecting pixels of a field color in a yuv Image
 */
class ColorBounding : public Filter
{
public:
  ColorBounding() : Filter("ColorBounding")
  {
  }

  virtual std::string getClassName() const override
  {
    return "ColorBounding";
  }

  /**
   * YUV limits for field detection. In public so they are accessible from other
   * filters.
   */
  ParamInt minY, maxY, minU, maxU, minV, maxV;

protected:
  /**
   * @Inherit
   */
  virtual void process() override;

  virtual void setParameters() override;

private:
};
}  // namespace Filters
}  // namespace Vision
