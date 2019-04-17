#include "GroundTruthProvider.hpp"

namespace Vision
{
namespace Filters
{
GroundTruthProvider::GroundTruthProvider() : imgIndex(0), outputPrefix("")
{
}

void GroundTruthProvider::setParameters()
{
  writeEnabled = ParamInt(1, 0, 1);
  patchSize = ParamInt(16, 1, 512);

  params()->define<ParamInt>("writeEnabled", &writeEnabled);
  params()->define<ParamInt>("patchSize", &patchSize);
}

Json::Value GroundTruthProvider::toJson() const
{
  Json::Value v = Filter::toJson();
  v["outputPrefix"] = outputPrefix;
  return v;
}

void GroundTruthProvider::fromJson(const Json::Value& v, const std::string& dir_name)
{
  Filter::fromJson(v, dir_name);
  rhoban_utils::tryRead(v, "outputPrefix", &outputPrefix);
}

void GroundTruthProvider::dumpImg(const std::vector<cv::Rect_<float>>& rois)
{
  if (writeEnabled <= 0)
  {
    return;
  }
  // TODO: fill with 0 as in printf
  std::string img_annotation_path = outputPrefix + "img_" + std::to_string(imgIndex) + ".json";
  rhoban_utils::writeJson(getImgAnnotation(rois), img_annotation_path);
  int patchIndex = 0;
  for (const cv::Rect_<float>& roi : rois)
  {
    // TODO: fill with 0 as in printf
    std::string patch_name = "patch_" + std::to_string(imgIndex) + "_" + std::to_string(patchIndex);
    dumpROI(roi, outputPrefix + patch_name + ".png");
    rhoban_utils::writeJson(getPatchAnnotation(roi), outputPrefix + patch_name + ".png");
    patchIndex++;
  }
}

void GroundTruthProvider::dumpROI(const cv::Rect_<float>& roi, const std::string& output);
{
  const cv::Mat& src_img = getSourceImg();
  cv::Mat raw_patch(src_img, cv::Rect(roi));
  cv::Mat rescaled_patch;
  cv::resize(raw_patch, rescaled_patch, cv::Size(patchSize, patchSize));
  cv::imwrite(output.c_str(), rescaled_patch);
}

Json::Value GroundTruthProvider::getPatchAnnotation(const cv::Rect_<float>& roi) const
{
  Json::Value v;
  for (const auto& entry : annotations)
  {
    std::string type_name = entry.first;
    for (size_t idx = 0; idx < entry.second.size(); idx++)
    {
      if (roi.contains(entry.second.center))
      {
        v[type_name][idx]["distance"] = entry.second.distance;
        v[type_name][idx]["center"][0] = entry.second.center.x;
        v[type_name][idx]["center"][1] = entry.second.center.y;
      }
    }
  }
  return v;
}

Json::Value GroundTruthProvider::getImgAnnotation(const std::vector<cv::Rect_<float>>& rois)
{
  Json::Value v;
  for (const auto& entry : annotations)
  {
    std::string type_name = entry.first;
    bool inside_roi = false;
    for (const cv::Rect_<float>& roi : rois)
    {
      if (roi.contains(entry.second.center))
      {
        inside_roi = true;
        break;
      }
    }
    for (size_t idx = 0; idx < entry.second.size(); idx++)
    {
      v[type_name][idx]["inside_roi"] = inside_roi;
      v[type_name][idx]["distance"] = entry.second.distance;
      v[type_name][idx]["center"][0] = entry.second.center.x;
      v[type_name][idx]["center"][1] = entry.second.center.y;
    }
  }
  return v;
}

}  // namespace Filters
}  // namespace Vision
