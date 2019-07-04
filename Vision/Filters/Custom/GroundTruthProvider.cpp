#include "GroundTruthProvider.hpp"

#include <hl_communication/utils.h>
#include <CameraState/CameraState.hpp>
#include <robocup_referee/constants.h>
#include <rhoban_utils/logging/logger.h>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <rhoban_utils/util.h>

using namespace hl_communication;
using hl_monitoring::Field;
using robocup_referee::Constants;

static rhoban_utils::Logger logger("GroundTruthProvider");

namespace Vision
{
namespace Filters
{
GroundTruthProvider::GroundTruthProvider()
  : Filter("GroundTruthProvider"), imgIndex(0), outputPrefix(""), labellingManager(Constants::field.ball_radius)
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
  writeEnabled = ParamInt(0, 0, 3);
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
  v["extractionMode"] = extractionMode;
  v["relativePosePath"] = relativePosePath;
  v["labellingPath"] = labellingPath;
  return v;
}

void GroundTruthProvider::fromJson(const Json::Value& v, const std::string& dir_name)
{
  Filter::fromJson(v, dir_name);
  rhoban_utils::tryRead(v, "outputPrefix", &outputPrefix);
  rhoban_utils::tryRead(v, "extractionMode", &extractionMode);
  rhoban_utils::tryRead(v, "relativePosePath", &relativePosePath);
  rhoban_utils::tryRead(v, "labellingPath", &labellingPath);
  if (extractionMode == "labels")
  {
    hl_communication::VideoMetaInformation video_meta;
    hl_communication::readFromFile(relativePosePath, &video_meta);
    labellingManager.importMetaData(video_meta);
    hl_communication::GameLabelCollection labels;
    hl_communication::readFromFile(labellingPath, &labels);
    labellingManager.importLabels(labels);
    labellingManager.summarize(&std::cout);
    labellingManager.sync();
  }
}

void GroundTruthProvider::updateAnnotations()
{
  annotations.clear();
  if (extractionMode == "vive")
    extractViveAnnotations();
  else if (extractionMode == "labels")
    extractLabelsAnnotations();
  else
    throw std::logic_error(DEBUG_INFO + " unknown extractionMode: '" + extractionMode + "'");
}

std::map<std::string, std::vector<Eigen::Vector3d>> GroundTruthProvider::getFieldFeaturesByType()
{
  std::map<std::string, std::vector<Eigen::Vector3d>> field_positions_by_type;
  for (const auto& entry : Constants::field.getPointsOfInterestByType())
  {
    std::string feature_name = Field::poiType2String(entry.first);
    for (const cv::Point3f& p : entry.second)
    {
      field_positions_by_type[feature_name].push_back(cv2Eigen(p));
    }
  }
  return field_positions_by_type;
}

void GroundTruthProvider::extractLabelsAnnotations()
{
  if (getCS().frame_status != FrameStatus::MOVING)
  {
    logger.warning("%s: Invalid frame status", DEBUG_INFO.c_str());
    return;
  }
  std::map<std::string, std::vector<Eigen::Vector3d>> field_positions_by_type = getFieldFeaturesByType();
  // Conversion to annotations
  Eigen::Affine3d camera_from_field = labellingManager.getCameraPose(getCS().source_id, getCS().utc_ts);
  for (const auto& entry : field_positions_by_type)
  {
    std::string feature_name = entry.first;
    for (const Eigen::Vector3d& field_pos : entry.second)
    {
      try
      {
        // TODO: functions allowing to retrieve the point without
        // risking to throw exception should be available
        Eigen::Vector3d camera_pos = camera_from_field * field_pos;
        Annotation a;
        a.distance = (camera_pos).norm();
        a.center = getCS().getCameraModel().getImgFromObject(eigen2CV(camera_pos));
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
  // Adding balls to entries
  for (const auto& entry : labellingManager.getBalls(getCS().utc_ts))
  {
    Eigen::Vector3d ball_in_field = entry.second;
    Eigen::Vector3d ball_in_camera = camera_from_field * ball_in_field;
    try
    {
      // TODO: functions allowing to retrieve the point without
      // risking to throw exception should be available
      Annotation a;
      a.distance = (ball_in_camera).norm();
      a.center = getCS().getCameraModel().getImgFromObject(eigen2CV(ball_in_camera));
      // Only include annotation if center is inside image
      if (cv::Rect(0, 0, img().cols, img().rows).contains(a.center))
      {
        annotations["Ball"].push_back(a);
      }
    }
    catch (const std::runtime_error& exc)
    {
    }
  }
  // TODO: remove self from robots?
  const hl_communication::RobotIdentifier& my_id = getCS().source_id.robot_source().robot_id();
  for (const auto& entry : labellingManager.getRobots(getCS().utc_ts))
  {
    if (my_id == entry.first)
      continue;
    Eigen::Vector3d robot_in_field = entry.second;
    Eigen::Vector3d robot_in_camera = camera_from_field * robot_in_field;
    try
    {
      // TODO: functions allowing to retrieve the point without
      // risking to throw exception should be available
      Annotation a;
      a.distance = (robot_in_camera).norm();
      a.center = getCS().getCameraModel().getImgFromObject(eigen2CV(robot_in_camera));
      // Only include annotation if center is inside image
      if (cv::Rect(0, 0, img().cols, img().rows).contains(a.center))
      {
        annotations["Robot"].push_back(a);
      }
    }
    catch (const std::runtime_error& exc)
    {
    }
  }
}

void GroundTruthProvider::extractViveAnnotations()
{
  // In case pose of the camera in the field is not available, no annotation are available
  if (!getCS().has_camera_field_transform)
  {
    logger.log("No camera field transform");
    return;
  }
  std::map<std::string, std::vector<Eigen::Vector3d>> field_positions_by_type = getFieldFeaturesByType();
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
          cv::Scalar color(255, 0, 255);
          if (entry.first == "Ball")
            color = cv::Scalar(0, 0, 255);
          else if (entry.first == "Robot")
            color = cv::Scalar(255, 0, 0);
          int radius = 3;
          cv::circle(img(), a.center, radius, color, CV_FILLED);
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
  std::string img_number_str = rhoban_utils::fixedSizeInt(imgIndex, std::pow(10, 6));
  if (writeEnabled == 2 || writeEnabled == 3)
  {
    std::string img_annotation_path = outputPrefix + "img_" + img_number_str + ".json";
    rhoban_utils::writeJson(getImgAnnotation(rois), img_annotation_path);
    const cv::Mat& src_img = getSourceImg();
    std::string imName = outputPrefix + "img_" + img_number_str + ".png";
    cv::imwrite(imName.c_str(), src_img);
  }
  if (writeEnabled == 1 || writeEnabled == 3)
  {
    int patchIndex = 0;
    for (const cv::Rect_<float>& roi : rois)
    {
      std::string patch_index_str = rhoban_utils::fixedSizeInt(patchIndex, 1000);
      std::string patch_name = "patch_" + img_number_str + "_" + patch_index_str;
      dumpROI(roi, outputPrefix + patch_name + ".png");
      rhoban_utils::writeJson(getPatchAnnotation(roi), outputPrefix + patch_name + ".json");
      patchIndex++;
    }
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
