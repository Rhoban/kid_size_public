#include "PatchProvider.hpp"

#include "Utils/ROITools.hpp"

#include <rhoban_utils/logging/logger.h>

#include <opencv2/imgproc/imgproc.hpp>

using namespace Vision::Utils;

static rhoban_utils::Logger logger("PatchProvider");

namespace Vision
{
namespace Filters
{
PatchProvider::PatchProvider(const std::string& name) : Filter(name)
{
}

void PatchProvider::setParameters()
{
  patchHeight = ParamInt(64, 2, 128);
  patchWidth = ParamInt(64, 2, 128);
  resizePolicy = ParamInt(1, 0, 2);

  params()->define<ParamInt>("patchHeight", &patchHeight);
  params()->define<ParamInt>("patchWidth", &patchWidth);
  params()->define<ParamInt>("resizePolicy", &resizePolicy);
}

void PatchProvider::clearPatches()
{
  patches.clear();
}

const std::vector<cv::Mat>& PatchProvider::getPatches() const
{
  return patches;
}

void PatchProvider::addPatches(const std::vector<cv::Rect>& rois, const cv::Mat& roi_img, const cv::Mat& src)
{
  std::vector<cv::Mat> result;
  for (const cv::Rect& roi : rois)
  {
    cv::Rect roi_src = resizeROI(roi, roi_img, src);
    cv::Mat raw_patch(src, roi_src);
    if (raw_patch.cols == 0 || raw_patch.rows == 0)
    {
      logger.log("Invalid patch size:\n"
                 "\tpatch_in_src: size is %dx%d, top-left is (%d,%d)\n"
                 "\tpatch_in_dst: size is %dx%d, top-left is (%d,%d)",
                 roi.height, roi.width, roi.x, roi.y, raw_patch.rows, raw_patch.cols, roi_src.x, roi_src.y);
      continue;
    }

    // Determine scale if resizing is required
    double col_scale = patchWidth / (double)raw_patch.cols;
    double row_scale = patchHeight / (double)raw_patch.rows;
    double scale = std::min(col_scale, row_scale);
    float roi_quality = 1.0;  // Quality are required but have no real meaning here
    switch ((int)resizePolicy)
    {
      case 0:  // Never resize
        patches.push_back(raw_patch);
        addRoi(roi_quality, toRotatedRect(roi_src));
        break;
      case 1:  // Resize patches which are too large
        if (scale >= 1)
        {
          patches.push_back(raw_patch);
          addRoi(roi_quality, toRotatedRect(roi_src));
          break;
        }
      case 2:
      {  // All patches have the same size
        cv::Size patch_size(raw_patch.cols * scale, raw_patch.rows * scale);
        // Obtaining scaled patch
        cv::Mat scaled_patch;
        cv::resize(raw_patch, scaled_patch, patch_size);
        patches.push_back(scaled_patch);
        addRoi(roi_quality, toRotatedRect(roi_src));
        break;
      }
    }
  }
}

void PatchProvider::addPatches(const std::vector<cv::RotatedRect>& rois, const cv::Mat& roi_img, const cv::Mat& src)
{
  std::vector<cv::Rect> rect_rois;
  for (const cv::RotatedRect& roi : rois)
  {
    rect_rois.push_back(cropRect(toRect(roi), roi_img));
  }
  addPatches(rect_rois, roi_img, src);
}

}  // namespace Filters
}  // namespace Vision
