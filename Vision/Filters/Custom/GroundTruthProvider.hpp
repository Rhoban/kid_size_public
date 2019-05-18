#pragma once

#include "Filters/Filter.hpp"

#include <opencv2/videoio.hpp>
namespace Vision
{
namespace Filters
{
/**
 * This class allows to extract patches from image along with meta-data indicating their content based on an expert
 * camera pose and on experts positions for the ball if available. The main purpose is to use it to extract labelled
 * information in order to train neural networks.
 */
class GroundTruthProvider : public Filter
{
public:
  struct Annotation
  {
    /**
     * Distance of the object toward the camera in meters
     */
    double distance;
    /**
     * Position of the object center inside the image [px]
     */
    cv::Point2f center;
  };

  GroundTruthProvider();
  ~GroundTruthProvider();

  Json::Value toJson() const override;
  void fromJson(const Json::Value& v, const std::string& dir_name) override;

protected:
  virtual void process() override;
  void setParameters() override;
  /**
   * Uses information from the camera state to update the map of annotations in the image
   */
  void updateAnnotations();

  /**
   * Updates output image depending
   */
  void tagImg();

  /**
   * Main specialization of this class is achieved through choice of the method used to generate the regions of interest
   */
  virtual std::vector<cv::Rect_<float>> generateROIs() = 0;

  /**
   * Return the image from which patches are extracted
   */
  virtual const cv::Mat& getSourceImg() = 0;

  /**
   * Saves the following content:
   * - a single img_<img_idx>.json annotation file
   * - multiple patch_<img_idx>_<patch_idx>.png -> images from the patch
   * - multiple patch_<img_idx>_<patch_idx>.json -> annotation files for the patches
   */
  void dumpImg(const std::vector<cv::Rect_<float>>& rois);

  void dumpROI(const cv::Rect_<float>& roi, const std::string& output);

  /**
   * Returns a Json node containing a map of element_types toward their patch properties (i.e. center in image,
   * distance)
   */
  Json::Value getPatchAnnotation(const cv::Rect_<float>& roi);

  /**
   * Return a Json node containing a map of element_types toward their properties in the image (i.e. included in an roi,
   * distance)
   */
  Json::Value getImgAnnotation(const std::vector<cv::Rect_<float>>& rois);

  /**
   * Current index of the image (to synchronize annotation and patches)
   */
  int imgIndex;

  /**
   * Output path prefix for all created files. Created if it does not exist
   */
  std::string outputPrefix;

  /**
   * 0: no data are written
   * 1: patches and img are written to the disk
   */
  ParamInt writeEnabled;

  /**
   * Before being written, all patches are resized to a square image of [patch_size,patch_size]
   */
  ParamInt patchSize;

  /**
   * Detailed of writing
   * 0: output = input
   * 1: annotation centers are highlighted
   */
  ParamInt tagLevel;

  /**
   * Stores the points of interests of an image ordered by type descriptor
   */
  std::map<std::string, std::vector<Annotation>> annotations;
};
}  // namespace Filters
}  // namespace Vision
