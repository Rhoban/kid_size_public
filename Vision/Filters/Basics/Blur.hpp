#ifndef VISION_FILTERS_BLUR_HPP
#define VISION_FILTERS_BLUR_HPP

#include "Filters/Filter.hpp"

namespace Vision {
namespace Filters {

/**
 * Blur
 *
 * Blur the image with a kernel of a given width and height
 */
class Blur : public Filter {
public:
  Blur() : Filter("Blur") {}

  /**
   * Initialization with filter name, filter source
   */
  Blur(const std::string &name, const std::string &source,
       Frequency::type frequency = Frequency::Auto);

  virtual std::string getClassName() const override { return "Blur"; }

protected:
  /**
   * @Inherit
   */
  virtual void process() override;

  virtual void setParameters() override;

private:
  ParamInt kWidth;
  ParamInt kHeight;
};
}
}

#endif
