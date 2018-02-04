#ifndef VISION_FILTERS_RESCALE_HPP
#define VISION_FILTERS_RESCALE_HPP

#include "Filters/Filter.hpp"

namespace Vision {
namespace Filters {

/**
 * Rescale
 *
 * Divide the width and the height of the given image by a ratio
 */
class Rescale : public Filter {
public:
  Rescale() : Filter("Rescale") {}

  /**
   * Initialization with filter name, filter source
   */
  Rescale(const std::string &name, const std::string &source,
          Frequency::type frequency = Frequency::Auto);

  virtual std::string getClassName() const override { return "Rescale"; }

protected:
  /**
   * @Inherit
   */
  virtual void process() override;

  virtual void setParameters() override;

private:
  ParamFloat _ratio;
};
}
}

#endif
