#ifndef WHITE_LINES_HPP
#define WHITE_LINES_HPP

#include "Filters/Filter.hpp"

namespace Vision {
namespace Filters {

/**
 * Recognize Border of the Field and compute the clipping
 */
class WhiteLines : public Filter {

 public:
  WhiteLines();
  WhiteLines(const std::string &name, const std::string &source,
	      Frequency::type frequency = Frequency::Auto);
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


#endif /* WHITE_LINES_HPP */
