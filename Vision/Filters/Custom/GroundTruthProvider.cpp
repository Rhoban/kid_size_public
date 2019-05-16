#include "GroundTruthProvider.hpp"

#include <CameraState/CameraState.hpp>
#include <robocup_referee/constants.h>
#include <rhoban_utils/logging/logger.h>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <rhoban_utils/util.h>

using hl_monitoring::Field;
using robocup_referee::Constants;

static rhoban_utils::Logger logger("GroundTruthProvider");

namespace Vision
{
namespace Filters
{
GroundTruthProvider::GroundTruthProvider() : Filter("GroundTruthProvider"), imgIndex(0), outputPrefix("")
{
}

GroundTruthProvider::~GroundTruthProvider()
{
}

void GroundTruthProvider::process()
{
  img() = getSourceImg();

  std::vector<cv::Rect_<float>> rois = generateROIs();
  updateAnnotations();
  tagImg();
  dumpImg(rois);
  imgIndex++;
}

void GroundTruthProvider::setParameters()
{
  writeEnabled = ParamInt(1, 0, 1);
  patchSize = ParamInt(32, 1, 512);
  tagLevel = ParamInt(1, 0, 1);

  params()->define<ParamInt>("writeEnabled", &writeEnabled);
  params()->define<ParamInt>("patchSize", &patchSize);
  params()->define<ParamInt>("tagLevel", &tagLevel);
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

void GroundTruthProvider::updateAnnotations()
{
  annotations.clear();
  // In case pose of the camera in the field is not available, no annotation are available
  if (!getCS().has_camera_field_transform)
  {
    logger.log("No camera field transform");
    return;
  }
  std::map<std::string, std::vector<Eigen::Vector3d>> field_positions_by_type;
  for (const auto& entry : Constants::field.getPointsOfInterestByType())
  {
    std::string feature_name = Field::poiType2String(entry.first);
    for (const cv::Point3f& p : entry.second)
    {
      field_positions_by_type[feature_name].push_back(cv2Eigen(p));
    }
  }
  for (const Eigen::Vector3d& ball_pos : getCS().vive_balls_in_field)
  {
    field_positions_by_type["Ball"].push_back(ball_pos);
  }
  for (const Eigen::Vector3d& tracker_pos : getCS().vive_trackers_in_field)
  {
    // Project tracker on ground
    Eigen::Vector3d robot_pos = tracker_pos;
    robot_pos.z() = 0;
    field_positions_by_type["Robot"].push_back(robot_pos);
  }
  // Now add all features to annotation collection
  for (const auto& entry : field_positions_by_type)
  {
    std::string feature_name = entry.first;
    for (const Eigen::Vector3d& field_pos : entry.second)
    {
      try
      {
        // TODO: functions allowing to retrieve the point without risking to throw exception should be available
        Annotation a;
        a.distance = (getCS().camera_from_field * field_pos).norm();
        a.center = getCS().imgFromFieldPosition(field_pos);
        // Only include annotation if center is inside image
        if (cv::Rect(0, 0, img().cols, img().rows).contains(a.center))
        {
          annotations[feature_name].push_back(a);
        }
      }
      catch (const std::runtime_error& exc)
      {
      }
    }
  }
}

void GroundTruthProvider::tagImg()
{
  switch (tagLevel)
  {
    case 0:
      return;
    case 1:
      img() = getSourceImg().clone();
      for (const auto& entry : annotations)
      {
        for (const Annotation& a : entry.second)
        {
          cv::circle(img(), a.center, 5, cv::Scalar(255, 0, 255), CV_FILLED);
        }
      }
  }
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

  const cv::Mat& src_img = getSourceImg();
  std::string imName = outputPrefix + "img_" + std::to_string(imgIndex) + ".png";
  cv::imwrite(imName.c_str(), src_img);

  int patchIndex = 0;
  for (const cv::Rect_<float>& roi : rois)
  {
    // TODO: fill with 0 as in printf
    std::string patch_name = "patch_" + std::to_string(imgIndex) + "_" + std::to_string(patchIndex);
    dumpROI(roi, outputPrefix + patch_name + ".png");
    rhoban_utils::writeJson(getPatchAnnotation(roi), outputPrefix + patch_name + ".json");
    patchIndex++;
  }
}

void GroundTruthProvider::dumpROI(const cv::Rect_<float>& roi, const std::string& output)
{
  const cv::Mat& src_img = getSourceImg();
  // logger.log("Dumping ROI: (%f,%f) to (%f,%f)", roi.tl().x, roi.tl().y, roi.br().x, roi.br().y);
  cv::Mat raw_patch(src_img, cv::Rect(roi));
  cv::Mat rescaled_patch;
  cv::resize(raw_patch, rescaled_patch, cv::Size(patchSize, patchSize));
  cv::imwrite(output.c_str(), rescaled_patch);
}

Json::Value GroundTruthProvider::getPatchAnnotation(const cv::Rect_<float>& roi)
{
  Json::Value v;
  for (const auto& entry : annotations)
  {
    std::string type_name = entry.first;
    for (int idx = 0; idx < (int)entry.second.size(); idx++)
    {
      const Annotation& annotation = entry.second[idx];
      if (roi.contains(entry.second[idx].center))
      {
        double ratio = patchSize / (double)roi.size().width;
        cv::Point2f patch_pos = (annotation.center - roi.tl()) * ratio;
        v[type_name][idx]["distance"] = annotation.distance;
        v[type_name][idx]["center"][0] = patch_pos.x;
        v[type_name][idx]["center"][1] = patch_pos.y;
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
    for (int idx = 0; idx < (int)entry.second.size(); idx++)
    {
      const Annotation& annotation = entry.second[idx];
      bool inside_roi = false;
      for (const cv::Rect_<float>& roi : rois)
      {
        if (roi.contains(annotation.center))
        {
          inside_roi = true;
          break;
        }
      }
      v[type_name][idx]["inside_roi"] = inside_roi;
      v[type_name][idx]["distance"] = annotation.distance;
      v[type_name][idx]["center"][0] = annotation.center.x;
      v[type_name][idx]["center"][1] = annotation.center.y;
    }
  }
  return v;
}

}  // namespace Filters
}  // namespace Vision
