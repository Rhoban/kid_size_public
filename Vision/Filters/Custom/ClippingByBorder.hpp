#pragma once

#include "Filters/Filter.hpp"

namespace Vision {
namespace Filters {

/**
 * Recognize Border of the Field and compute the clipping
 */
class ClippingByBorder : public Filter {

 public:
  ClippingByBorder();
  ~ClippingByBorder();

  virtual std::string getClassName() const override { return "ClippingByBorder"; }
  virtual int expectedDependencies() const override { return 1; }

 protected:
  /**
   * @Inherit
   */
  virtual void process() override;
  virtual void setParameters() override;

  /* 0 or 1 : activate debug info */
  ParamInt debug_output;
  /* the screen ratio of the vertical shift */
  ParamFloat vshift_ratio;
};

}
}
