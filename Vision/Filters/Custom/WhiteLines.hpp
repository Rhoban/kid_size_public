#pragma once

#include "Filters/Filter.hpp"

namespace Vision {
namespace Filters {

/**
 * Recognize Border of the Field and compute the clipping
 */
class WhiteLines : public Filter {

 public:
  WhiteLines();
  ~WhiteLines();

  virtual std::string getClassName() const override { return "WhiteLines"; }
  virtual int expectedDependencies() const override { return 3; }

 protected:
  /**
   * @Inherit
   */
  virtual void process() override;
  virtual void setParameters() override;

};

}
}
