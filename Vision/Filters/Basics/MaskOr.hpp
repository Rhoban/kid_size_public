#ifndef VISION_FILTERS_MASKOR_HPP
#define VISION_FILTERS_MASKOR_HPP

#include "Filters/Filter.hpp"

namespace Vision {
namespace Filters {

/**
 * MaskOr
 * Does a logic OR between the inputs
 */
class MaskOr : public Filter {
public:
  MaskOr() : Filter("MaskOr") {}

  /**
   * Initialization with filter, src and mask names.
   */
  MaskOr(const std::string &name, const std::string &src,
         const std::string &mask, Frequency::type frequency = Frequency::Auto);

  virtual std::string getClassName() const override { return "MaskOr"; }
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
