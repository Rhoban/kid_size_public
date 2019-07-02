#pragma once

#include "Filters/Filter.hpp"

namespace Vision
{
namespace Filters
{
/**
 * Takes as input a filter which provides ROI and randomize their size and
 * position to create more diversity
 *
 * There are two guarantees on the number of ROI returned:
 * - The total will be at most maxROI (if inputROI is more than maxROI, some ROI will be lost)
 * - The total number of ROI present for each input roi is at most maxROIPerInput
 *
 * dependency[0]: ROI provider
 */
class ROIRandomizer : public Filter
{
public:
  ROIRandomizer();
  std::string getClassName() const override;
  int expectedDependencies() const override;

protected:
  void process() override;
  void setParameters() override;

  void randomizeROI(cv::Rect_<float>* roi);

  void updateAlterationLimits();

private:
  /**
   * The number generator
   */
  std::default_random_engine engine;
  /**
   * The maximum number of ROI as output of the filter
   */
  ParamInt maxROI;
  /**
   * The maximum number of ROI generated for an input ROI
   */
  ParamInt maxROIPerInput;
  ParamFloat sizeFixedNoiseMax;
  ParamFloat sizePropNoiseMax;
  ParamFloat posFixedNoiseMax;
  ParamFloat posPropNoiseMax;
  ParamInt verbose;

  /**
   * row_1 : size_fixed
   * row_2 : size_prop
   * row_3 : pos_fixed_x
   * row_4 : pos_prop_x
   * row_5 : pos_fixed_y
   * row_6 : pos_prop_y
   */
  Eigen::MatrixXd alteration_limits;
};
}  // namespace Filters
}  // namespace Vision
