#include "RoiToPatches.hpp"

#include "Utils/ROITools.hpp"
#include "Utils/RotatedRectUtils.hpp"

namespace Vision
{
namespace Filters
{
RoiToPatches::RoiToPatches() : PatchProvider("RoiToPatches")
{
}

std::string RoiToPatches::getClassName() const
{
  return "RoiToPatches";
}

void RoiToPatches::setParameters()
{
  PatchProvider::setParameters();

  tagLevel = ParamInt(1, 0, 1);

  params()->define<ParamInt>("tagLevel", &tagLevel);
}

int RoiToPatches::expectedDependencies() const
{
  return 2;
}

void RoiToPatches::process()
{
  const Filter& src = getDependency(_dependencies[0]);
  const Filter& roi_provider = getDependency(_dependencies[1]);

  const cv::Mat& src_img = *(src.getImg());
  const cv::Mat& roi_img = *(roi_provider.getImg());

  std::vector<cv::RotatedRect> rois;
  for (const std::pair<float, cv::RotatedRect>& scored_roi : roi_provider.getRois())
  {
    rois.push_back(scored_roi.second);
  }

  patches.clear();
  clearRois();
  addPatches(rois, roi_img, src_img);

  // If no tagging,  just copy reference
  if (tagLevel == 0)
  {
    img() = src_img;
  }
  else
  {
    // Rescale rois
    std::vector<cv::RotatedRect> rescaled_rois;
    for (const std::pair<float, cv::RotatedRect>& scored_roi : roi_provider.getRois())
    {
      rescaled_rois.push_back(Utils::resizeROI(scored_roi.second, roi_img, src_img));
    }
    img() = src_img;
    // Currently disabled
    //// Clone src_img and draw on it
    // cv::Mat tmp_img = src_img.clone();
    // drawRotatedRectangles(tmp_img, rescaled_rois, cv::Scalar(0,0,255));
    // img() = tmp_img;
  }
}

}  // namespace Filters
}  // namespace Vision
