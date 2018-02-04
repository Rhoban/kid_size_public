#ifndef VISION_FILTERS_NORM_HPP
#define VISION_FILTERS_NORM_HPP

#include "Filters/Filter.hpp"

namespace Vision {
namespace Filters {

/**
 * Norm
 *
 * Take a multi-channel image and return a 1 channel image where the value of
 * dst(x,y) is the norm of all the channels from the source image
 */
class Norm : public Filter {
public:
  Norm() : Filter("Norm") {}

  /**
   * Initialization with filter name, filter source
   */
  Norm(const std::string &name, const std::string &source,
       Frequency::type frequency = Frequency::Auto);

  virtual std::string getClassName() const override { return "Norm"; }

protected:
  /**
   * @Inherit
   */
  virtual void process() override;
};
}
}

#endif
