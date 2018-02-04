#ifndef VISION_FILTERS_DIFF_HPP
#define VISION_FILTERS_DIFF_HPP

#include "Filters/Filter.hpp"

namespace Vision {
namespace Filters {

/**
 * Diff
 *
 * Calculates the differences between two images A and B
 * (Order matters and if A[i,j] < B[i,j], then R[i,j] = 0)
 */
class Diff : public Filter {
public:
  Diff() : Filter("Diff") {}

  /**
   * Initialization with filter name, A and B sources (A-B) and frequency
   */
  Diff(const std::string &name, const std::string &A, const std::string &B,
       Frequency::type frequency = Frequency::Auto);

  virtual std::string getClassName() const override { return "Diff"; }
  virtual int expectedDependencies() const override { return 2; }

protected:
  /**
   * @Inherit
   */
  virtual void process() override;
};
}
}

#endif
