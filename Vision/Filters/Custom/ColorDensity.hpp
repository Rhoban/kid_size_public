#pragma once

#include "Filters/Filter.hpp"

namespace Vision
{
namespace Filters
{
/**
 * Recognize Border of the Field and compute the clipping
 */
class ColorDensity : public Filter
{
public:
  ColorDensity();
  ~ColorDensity();

  virtual std::string getClassName() const override
  {
    return "ColorDensity";
  }
  virtual int expectedDependencies() const override
  {
    return 1;
  }

private:
  /* the size of the side of the (squared) kernel
     used to compute the density */
  ParamInt kernel_size;
  /* The resizing factor (0.5 makes the image 2 times smaller) */
  ParamFloat resize_factor;

protected:
  /**
   * @Inherit
   */
  virtual void process() override;
  virtual void setParameters() override;
};

}  // namespace Filters
}  // namespace Vision
