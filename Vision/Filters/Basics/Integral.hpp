#ifndef VISION_FILTERS_INTEGRAL_HPP
#define VISION_FILTERS_INTEGRAL_HPP

#include "Filters/Filter.hpp"

namespace Vision
{
namespace Filters
{
/**
 * Integral
 *
 * Outputs the integral image formed by the single channel input image
 */
class Integral : public Filter
{
public:
  Integral() : Filter("Integral")
  {
  }

  virtual std::string getClassName() const override
  {
    return "Integral";
  }

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

#endif
