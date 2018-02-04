#ifndef VISION_FILTERS_THRESHOLD_HPP
#define VISION_FILTERS_THRESHOLD_HPP

#include "Filters/Filter.hpp"

namespace Vision {
namespace Filters {

/**
 * Threshold
 *
 * Apply a threshold on a given image, different types of threshold might be
 * applied
 * WARNING: This filter is not adapted for working with floating images
 */
class Threshold : public Filter {
public:
  Threshold() : Filter("Threshold") {}

  /**
   * Initialization with filter name, filter source
   */
  Threshold(const std::string &name, const std::string &source,
            Frequency::type frequency = Frequency::Auto);

  virtual std::string getClassName() const override { return "Threshold"; }

protected:
  /**
   * @Inherit
   */
  virtual void process() override;

  virtual void setParameters() override;

private:
  ParamInt threshold;
  /**
   * Threshold Type:
   * 0: Binary
   * 1: Binary Inverted
   * 2: Truncate
   * 3: To Zero
   * 4: To Zero Inverted
   */
  ParamInt thresholdType;
};
}
}

#endif
