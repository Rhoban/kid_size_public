#ifndef VISION_FILTERS_ABSDIFF_HPP
#define VISION_FILTERS_ABSDIFF_HPP

#include "Filters/Filter.hpp"

namespace Vision
{
namespace Filters
{
/**
 * AbsDiff
 *
 * Calculates the absolute difference between two images A and B
 * (order do not matter)
 */
class AbsDiff : public Filter
{
public:
  AbsDiff() : Filter("absDiff")
  {
  }

protected:
  /**
   * @Inherit
   */
  virtual void process() override;

  virtual std::string getClassName() const override
  {
    return "AbsDiff";
  }
  virtual int expectedDependencies() const override
  {
    return 2;
  }
};
}  // namespace Filters
}  // namespace Vision

#endif
