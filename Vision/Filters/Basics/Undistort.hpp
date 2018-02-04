#ifndef VISION_FILTERS_UNDISTORT_HPP
#define VISION_FILTERS_UNDISTORT_HPP

#include "Filters/Filter.hpp"
#include "opencv2/ocl/ocl.hpp"


namespace Vision {
namespace Filters {

/**
 * Undistort
 * Corrects an image
 */
class Undistort : public Filter {
public:
  Undistort();

  /**
   * Initialization with filter name, filter source
   */
  Undistort(const std::string &name, const std::string &source,
            Frequency::type frequency = Frequency::Auto);

  virtual std::string getClassName() const override { return "Undistort"; }

  // Given the position of a pixel p in the undistorded image, return its
  // predecessor
  cv::Point predecessor(const cv::Point &p) const;

  cv::Mat undistortOneShot(cv::Mat);

protected:
  /**
   * @Inherit
   */
  virtual void process() override;

  virtual void setParameters() override;

private:
  bool _first = true;
  cv::Mat _map1, _map2, _map1Inverted;
  cv::ocl::oclMat map1, map2;
};
}
}

#endif
