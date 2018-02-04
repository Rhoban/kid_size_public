#ifndef VISION_FILTERS_INVERT_HPP
#define VISION_FILTERS_INVERT_HPP

#include "Filters/Filter.hpp"

namespace Vision {
namespace Filters {

/**
 * Invert
 *
 * Invert a CV_8U image (dst[i,j] = 255 - src[i,j])
 */
class Invert : public Filter {
public:
  Invert() : Filter("Invert") {}

  /**
   * Initialization with filter name, filter source
   */
  Invert(const std::string &name, const std::string &source,
         Frequency::type frequency = Frequency::Auto);

  virtual std::string getClassName() const override { return "Invert"; }

protected:
  /**
   * @Inherit
   */
  virtual void process() override;
};
}
}

#endif
