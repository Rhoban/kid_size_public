#ifndef VISION_FILTERS_ABSDIFF_HPP
#define VISION_FILTERS_ABSDIFF_HPP

#include "Filters/Filter.hpp"

namespace Vision {
namespace Filters {

/**
 * AbsDiff
 *
 * Calculates the absolute difference between two images A and B
 * (order do not matter)
 */
class AbsDiff : public Filter {
public:
  AbsDiff() : Filter("absDiff") {}

  /**
   * Initialization with filter name, A and B sources abs(A-B) and
   * frequency
   */
  AbsDiff(const std::string &name, const std::string &A, const std::string &B,
          Frequency::type frequency = Frequency::Auto);

protected:
  /**
   * @Inherit
   */
  virtual void process() override;

  virtual std::string getClassName() const override { return "AbsDiff"; }
  virtual int expectedDependencies() const override { return 2; }
};
}
}

#endif
