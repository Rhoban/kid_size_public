#ifndef VISION_FILTERS_MULT_HPP
#define VISION_FILTERS_MULT_HPP

#include "Filters/Filter.hpp"

namespace Vision {
namespace Filters {

/**
 * Mult
 *
 * Multiply each channel of the image by the given ratio
 */
class Mult : public Filter {
public:
  Mult() : Filter("Mult") {}

  /**
   * Initialization with filter name, filter source
   */
  Mult(const std::string &name, const std::string &source,
       Frequency::type frequency = Frequency::Auto);

  virtual std::string getClassName() const override { return "Mult"; }

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
