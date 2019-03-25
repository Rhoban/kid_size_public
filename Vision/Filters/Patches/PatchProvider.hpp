#pragma once

#include "Filters/Filter.hpp"

namespace Vision
{
namespace Filters
{
/// A patch provider is a Filter producing one or several patches. Those patches
/// may further be used for classification.
/// Position and size of the patch are provided (using a relative scale based on
/// source image)
class PatchProvider : public Filter
{
public:
  PatchProvider(const std::string& name);

  virtual void setParameters() override;

  void clearPatches();

  const std::vector<cv::Mat>& getPatches() const;

  /// Add patches corresponding to rois to the list of existing patches. Also
  /// Add corresponding ROI in 'src' referential
  /// @param rois provided in roi_img referential
  /// @param roi_img The image from which 'rois' have been extracted
  /// @param src The image used to extract region of interests
  void addPatches(const std::vector<cv::Rect>& rois, const cv::Mat& roi_img, const cv::Mat& src);

  /// Transforms RotatedRect in Rect and then extract patches
  void addPatches(const std::vector<cv::RotatedRect>& rois, const cv::Mat& roi_img, const cv::Mat& src);

protected:
  /// The list of patches detected
  std::vector<cv::Mat> patches;

  /// Height of the patches provided
  ParamInt patchHeight;
  /// Width of the patches provided
  ParamInt patchWidth;

  /// How do we handle the cases where patch size is not inside bounds
  /// 0 - Never resize
  /// 1 - Resize patches larger than given size
  /// 2 - Resize all patches to the given size
  ParamInt resizePolicy;
};

}  // namespace Filters
}  // namespace Vision
