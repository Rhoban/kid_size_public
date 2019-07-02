#include "ROIRandomizer.hpp"

#include <Utils/ROITools.hpp>
#include <rhoban_random/tools.h>
#include <rhoban_utils/logging/logger.h>

static rhoban_utils::Logger logger("ROIRandomizer");

namespace Vision
{
namespace Filters
{
ROIRandomizer::ROIRandomizer() : Filter("ROIRandomizer"), alteration_limits(6, 2)
{
  engine = rhoban_random::getRandomEngine();
}

std::string ROIRandomizer::getClassName() const
{
  return "ROIRandomizer";
}

int ROIRandomizer::expectedDependencies() const
{
  return 1;
}

void ROIRandomizer::setParameters()
{
  maxROI = ParamInt(16, 0, 64);
  maxROIPerInput = ParamInt(8, 0, 16);
  sizeFixedNoiseMax = ParamFloat(5.0, 0.0, 20.0);
  sizePropNoiseMax = ParamFloat(0.2, 0.0, 1.0);
  posFixedNoiseMax = ParamFloat(5.0, 0.0, 20.0);
  posPropNoiseMax = ParamFloat(0.2, 0.0, 1.0);
  verbose = ParamInt(0, 0, 1);

  params()->define<ParamInt>("maxROI", &maxROI);
  params()->define<ParamInt>("maxROIPerInput", &maxROIPerInput);
  params()->define<ParamFloat>("sizeFixedNoiseMax", &sizeFixedNoiseMax);
  params()->define<ParamFloat>("sizePropNoiseMax", &sizePropNoiseMax);
  params()->define<ParamFloat>("posFixedNoiseMax", &posFixedNoiseMax);
  params()->define<ParamFloat>("posPropNoiseMax", &posPropNoiseMax);
  params()->define<ParamInt>("verbose", &verbose);
}

void ROIRandomizer::randomizeROI(cv::Rect_<float>* roi)
{
  // One alteration for x and the other for y
  Eigen::VectorXd alterations = rhoban_random::getUniformSample(alteration_limits, &engine);
  double old_size = roi->width;
  double new_size = alterations(0) + alterations(1) * old_size;
  roi->width = new_size;
  roi->height = new_size;
  double size_increase = new_size - old_size;
  roi->x += alterations(2) + alterations(3) * old_size - size_increase / 2;
  roi->y += alterations(4) + alterations(5) * old_size - size_increase / 2;
}

void ROIRandomizer::process()
{
  img() = *(getDependency(0).getImg());
  updateAlterationLimits();
  clearRois();
  std::vector<cv::Rect_<float>> input_rois;
  for (const auto& entry : getDependency(0).getRois())
  {
    input_rois.push_back(Utils::toRect(entry.second));
  }
  cv::Size img_size(img().cols, img().rows);

  int round = 0;
  if (verbose)
    logger.log("Nb rois_input : %d", input_rois.size());
  while (round < maxROIPerInput && _rois.size() < (size_t)maxROI)
  {
    int missing_rois = maxROI - _rois.size();
    if (verbose)
      logger.log("# ROUND %d | features_missing: %d", round, missing_rois);
    std::vector<size_t> roi_indices = rhoban_random::getUpToKDistinctFromN(missing_rois, input_rois.size(), &engine);
    for (size_t roi_idx : roi_indices)
    {
      cv::Rect_<float> new_roi = input_rois[roi_idx];
      // On first round, basic ROI are used
      if (round > 0)
      {
        randomizeROI(&new_roi);
      }
      if (Utils::isContained(new_roi, img_size))
      {
        addRoi(1.0, Utils::toRotatedRect(new_roi));
      }
    }
    round++;
  }
}

void ROIRandomizer::updateAlterationLimits()
{
  Eigen::VectorXd offset(6);
  offset(0) = sizeFixedNoiseMax;
  offset(1) = sizePropNoiseMax;
  offset(2) = posFixedNoiseMax;
  offset(3) = posPropNoiseMax;
  offset(4) = posFixedNoiseMax;
  offset(5) = posPropNoiseMax;
  Eigen::VectorXd mean(6);
  mean(0) = 0;
  mean(1) = 1;
  mean(2) = 0;
  mean(3) = 0;
  mean(4) = 0;
  mean(5) = 0;
  alteration_limits.col(0) = mean - offset;
  alteration_limits.col(1) = mean + offset;
}

}  // namespace Filters
}  // namespace Vision
