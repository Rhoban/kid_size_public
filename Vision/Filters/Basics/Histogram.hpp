#ifndef VISION_FILTERS_HISTOGRAM_HPP
#define VISION_FILTERS_HISTOGRAM_HPP

#include "Filters/Filter.hpp"

namespace Vision {
namespace Filters {

/**
 * Hisogram
 *
 * Separates the input image in 3 channels and draws an histogram for each of
 *them
 */
class Histogram : public Filter {
public:
  Histogram();

  /**
   * Initialization with filter name, filter source
   */
  Histogram(const std::string &name, const std::string &source,
            Frequency::type frequency = Frequency::Auto);

  virtual std::string getClassName() const override { return "Histogram"; }

protected:
  /**
   * @Inherit
   */
  virtual void process() override;

  virtual void setParameters() override;

private:
  ParamInt bins;
};
}
}

#endif
