#include "CameraState.hpp"
#include <iostream>

#include "Utils/HomogeneousTransform.hpp"
#include "services/DecisionService.h"
#include "services/ModelService.h"
#include "services/LocalisationService.h"
#include "services/RefereeService.h"
#include "services/ViveService.h"

#include <hl_communication/camera.pb.h>
#include <hl_communication/utils.h>
#include <rhoban_utils/util.h>
#include <rhoban_utils/logging/logger.h>

#include <rhoban_geometry/3d/plane.h>
#include <rhoban_geometry/3d/intersection.h>

#include <cmath>

#include <Eigen/StdVector>
#include <utility>
#include <vector>
#include <sstream>
#include <algorithm>

#include <robocup_referee/constants.h>

#include <string>

using namespace hl_communication;
using namespace hl_monitoring;
using namespace rhoban_geometry;
using namespace rhoban_utils;
using namespace robocup_referee;

static rhoban_utils::Logger logger("CameraState");

namespace Vision
{
namespace Utils
{
float CameraState::motor_delay(0);

CameraState::CameraState()
  : _moveScheduler(nullptr)
  , monotonic_ts(0.0)
  , utc_ts(0)
  , has_camera_field_transform(false)
  , clock_offset(0)
  , frame_status(FrameStatus::UNKNOWN_FRAME_STATUS)
{
}

CameraState::CameraState(MoveScheduler* moveScheduler) : CameraState()
{
  _moveScheduler = moveScheduler;
  _cameraModel = _moveScheduler->getServices()->model->cameraModel;
}

CameraState::CameraState(const IntrinsicParameters& camera_parameters, const FrameEntry& frame_entry,
                         const Pose3D& camera_from_self_pose, const Pose3D& camera_from_head_base_pose,
                         const hl_communication::VideoSourceID& source_id_, MoveScheduler* moveScheduler)
  : CameraState(moveScheduler)
{
  source_id = source_id_;
  importFromProtobuf(camera_parameters);
  importFromProtobuf(frame_entry);

  Eigen::Affine3d cameraFromSelf = getAffineFromProtobuf(camera_from_self_pose);
  worldToSelf = cameraFromSelf.inverse() * cameraToWorld.inverse();
  selfToWorld = worldToSelf.inverse();

  cameraFromHeadBase = getAffineFromProtobuf(camera_from_head_base_pose);

  // Removing correction if needed
  if (_moveScheduler == nullptr)
  {
    throw std::logic_error(DEBUG_INFO + " null movescheduler are not allowed anymore");
  }
  ModelService* modelService = _moveScheduler->getServices()->model;
  if (modelService->applyCorrectionInNonCorrectedReplay)
  {
    Eigen::Affine3d worldFromHeadBase = cameraToWorld * cameraFromHeadBase;
    cameraToWorld = modelService->applyCalibration(cameraToWorld, worldFromHeadBase, selfToWorld);
    worldToCamera = cameraToWorld.inverse();
    cameraFromHeadBase = worldToCamera * worldFromHeadBase;
  }
}

cv::Size CameraState::getImgSize() const
{
  return getCameraModel().getImgSize();
}

void CameraState::importFromProtobuf(const IntrinsicParameters& camera_parameters)
{
  _cameraModel.setCenter(Eigen::Vector2d(camera_parameters.center_x(), camera_parameters.center_y()));
  _cameraModel.setFocal(Eigen::Vector2d(camera_parameters.focal_x(), camera_parameters.focal_y()));
  _cameraModel.setImgWidth(camera_parameters.img_width());
  _cameraModel.setImgHeight(camera_parameters.img_height());
  if (camera_parameters.distortion_size() != 0)
  {
    Eigen::VectorXd distortion(camera_parameters.distortion_size());
    for (int i = 0; i < camera_parameters.distortion_size(); i++)
    {
      distortion(i) = camera_parameters.distortion(i);
    }
    _cameraModel.setDistortion(distortion);
  }
}

void CameraState::importFromProtobuf(const FrameEntry& src)
{
  monotonic_ts = ((double)src.monotonic_ts()) / std::pow(10, 6);
  utc_ts = src.utc_ts();
  worldToCamera = getAffineFromProtobuf(src.pose());
  cameraToWorld = worldToCamera.inverse();
  selfToWorld = Eigen::Affine3d::Identity();  // Protobuf does not store the position of the robot
  worldToSelf = selfToWorld.inverse();
  if (src.has_status())
  {
    frame_status = src.status();
  }
  else
  {
    frame_status = FrameStatus::UNKNOWN_FRAME_STATUS;
  }
}

void CameraState::exportToProtobuf(IntrinsicParameters* dst) const
{
  dst->set_focal_x(_cameraModel.getFocalX());
  dst->set_focal_y(_cameraModel.getFocalY());
  dst->set_center_x(_cameraModel.getCenterX());
  dst->set_center_y(_cameraModel.getCenterY());
  dst->set_img_width(_cameraModel.getImgWidth());
  dst->set_img_height(_cameraModel.getImgHeight());
  Eigen::VectorXd distortion = _cameraModel.getDistortionCoeffsAsEigen();
  dst->clear_distortion();
  for (int i = 0; i < distortion.size(); i++)
  {
    dst->add_distortion(distortion(i));
  }
}

void CameraState::exportToProtobuf(FrameEntry* dst) const
{
  dst->set_monotonic_ts((uint64_t)(monotonic_ts * std::pow(10, 6)));
  dst->set_utc_ts(utc_ts);
  setProtobufFromAffine(worldToCamera, dst->mutable_pose());
  dst->set_status(frame_status);
}

void CameraState::importHeader(const hl_communication::VideoMetaInformation& src)
{
  if (src.has_camera_parameters())
  {
    importFromProtobuf(src.camera_parameters());
  }
  if (src.has_source_id())
  {
    source_id.CopyFrom(src.source_id());
  }
}

void CameraState::exportHeader(hl_communication::VideoMetaInformation* dst) const
{
  exportToProtobuf(dst->mutable_camera_parameters());
  dst->mutable_source_id()->CopyFrom(source_id);
}

const rhoban::CameraModel& CameraState::getCameraModel() const

{
  return _cameraModel;
}

double CameraState::getSchedulerTS() const
{
  return getSchedulerTS(getTimeStamp());
}

double CameraState::getSchedulerTS(const rhoban_utils::TimeStamp& ts) const
{
  return ((double)ts.getTimeUS(false) / 1000.0 + motor_delay) / 1000.0;
}

void CameraState::updateInternalModel(const rhoban_utils::TimeStamp& ts)
{
  monotonic_ts = ts.getTimeUS(false);
  utc_ts = ts.getTimeUS(true);

  if (_moveScheduler != nullptr)
  {
    double scheduler_ts = getSchedulerTS(ts);
    ModelService* modelService = _moveScheduler->getServices()->model;
    ViveService* vive = _moveScheduler->getServices()->vive;
    DecisionService* decision = _moveScheduler->getServices()->decision;
    RefereeService* referee = _moveScheduler->getServices()->referee;

    source_id.Clear();
    RobotCameraIdentifier* identifier = source_id.mutable_robot_source();
    identifier->mutable_robot_id()->set_team_id(referee->teamId);
    identifier->mutable_robot_id()->set_robot_id(referee->id);
    identifier->set_camera_name("main");

    selfToWorld = modelService->selfToWorld(scheduler_ts);
    worldToCamera = modelService->cameraToWorld(scheduler_ts).inverse();
    _cameraModel = modelService->cameraModel;
    worldToSelf = selfToWorld.inverse();
    cameraToWorld = worldToCamera.inverse();
    cameraFromHeadBase = worldToCamera * modelService->headBaseToWorld(scheduler_ts).inverse();
    frame_status = decision->camera_status;
    // Update camera/field transform based on (by order of priority)
    // 1. Vive
    // 2. LocalisationService if field quality is good
    // 3. If nothing is available set info to false
    vive_balls_in_field.clear();
    vive_trackers_in_field.clear();
    if (vive->isActive())
    {
      try
      {
        camera_from_field = vive->getFieldToCamera(utc_ts, true);
        std::cout << "Vive based update: cameraPosInField: "
                  << (camera_from_field.inverse() * Eigen::Vector3d::Zero()).transpose() << std::endl;
        has_camera_field_transform = true;
        for (const Eigen::Vector3d& tagged_pos : vive->getTaggedPositions(utc_ts, true))
        {
          Eigen::Vector3d ball_pos = tagged_pos;
          ball_pos.z() = Constants::field.ball_radius;
          vive_balls_in_field.push_back(ball_pos);
        }
        for (const Eigen::Vector3d& pos : vive->getOthersTrackersPos(utc_ts, true))
        {
          vive_trackers_in_field.push_back(pos);
        }
      }
      catch (const std::out_of_range& exc)
      {
        has_camera_field_transform = false;
        camera_from_field = Eigen::Affine3d::Identity();
        logger.error("Failed to import transform from Vive: %s", exc.what());
      }
      catch (const std::runtime_error& exc)
      {
        has_camera_field_transform = false;
        camera_from_field = Eigen::Affine3d::Identity();
        logger.error("Failed to import transform from Vive: %s", exc.what());
      }
    }
    else if (decision->isFieldQualityGood)
    {
      LocalisationService* loc = _moveScheduler->getServices()->localisation;
      camera_from_field = worldToCamera * loc->world_from_field;
      has_camera_field_transform = true;
    }
    else
    {
      has_camera_field_transform = false;
      camera_from_field = Eigen::Affine3d::Identity();
    }
    field_from_camera = camera_from_field.inverse();
  }
  else
  {
    logger.warning("Not updating internal model (no moveScheduler available)");
  }
}

Angle CameraState::getTrunkYawInWorld()
{
  Eigen::Vector3d dirInWorld = selfToWorld.linear() * Eigen::Vector3d::UnitX();

  return Angle(rad2deg(atan2(dirInWorld(1), dirInWorld(0))));
}

cv::Point2f CameraState::robotPosFromImg(double imgX, double imgY) const
{
  cv::Point2f posInWorldCV = worldPosFromImg(imgX, imgY);

  Eigen::Vector3d posInWorld(posInWorldCV.x, posInWorldCV.y, 0);
  Eigen::Vector3d posInSelf = worldToSelf * posInWorld;

  return cv::Point2f(posInSelf(0), posInSelf(1));
}

cv::Point2f CameraState::worldPosFromImg(double imgX, double imgY) const
{
  Eigen::Vector3d posInWorld = posInWorldFromPixel(cv::Point2f(imgX, imgY));

  return cv::Point2f(posInWorld(0), posInWorld(1));
}

Eigen::Vector2d CameraState::getVecInSelf(const Eigen::Vector2d& vec_in_world) const
{
  Eigen::Vector3d src_in_world = Eigen::Vector3d::Zero();
  Eigen::Vector3d dst_in_world = Eigen::Vector3d::Zero();
  dst_in_world.segment(0, 2) = vec_in_world;

  Eigen::Vector3d src_in_self, dst_in_self;
  src_in_self = worldToSelf * src_in_world;
  dst_in_self = worldToSelf * dst_in_world;

  return (dst_in_self - src_in_self).segment(0, 2);
}

cv::Point2f CameraState::getPosInSelf(const cv::Point2f& pos_in_origin) const
{
  Eigen::Vector3d pos_in_self = worldToSelf * Eigen::Vector3d(pos_in_origin.x, pos_in_origin.y, 0);
  return cv::Point2f(pos_in_self(0), pos_in_self(1));
}

rhoban_geometry::PanTilt CameraState::robotPanTiltFromImg(double imgX, double imgY) const
{
  Eigen::Vector3d viewVectorInCamera = cv2Eigen(_cameraModel.getViewVectorFromImg(cv::Point2f(imgX, imgY)));
  Eigen::Vector3d viewVectorInSelf = worldToSelf.linear() * cameraToWorld.linear() * viewVectorInCamera;

  return rhoban_geometry::PanTilt(viewVectorInSelf);
}

Eigen::Vector3d CameraState::getWorldPosFromCamera(const Eigen::Vector3d& pos_camera) const
{
  return cameraToWorld * pos_camera;
}

Eigen::Vector3d CameraState::getSelfFromWorld(const Eigen::Vector3d& pos_world) const
{
  return worldToSelf * pos_world;
}

Eigen::Vector3d CameraState::getWorldFromSelf(const Eigen::Vector3d& pos_self) const
{
  return selfToWorld * pos_self;
}

Angle CameraState::getPitch()
{
  PanTilt panTilt(worldToSelf.linear() * cameraToWorld.linear() * Eigen::Vector3d::UnitZ());
  return panTilt.tilt;
}

Angle CameraState::getYaw()
{
  PanTilt panTilt(worldToSelf.linear() * cameraToWorld.linear() * Eigen::Vector3d::UnitZ());
  return panTilt.pan;
}

double CameraState::getHeight()
{
  // Getting height at camera origin
  double height = (cameraToWorld * Eigen::Vector3d::Zero())(2);
  if (height < 0)
  {
    height = 0;
  }
  return height;
}

cv::Point2f CameraState::imgXYFromWorldPosition(const cv::Point2f& p) const
{
  return imgXYFromWorldPosition(Eigen::Vector3d(p.x, p.y, 0));
}

cv::Point2f CameraState::imgXYFromWorldPosition(const Eigen::Vector3d& posInWorld) const
{
  Eigen::Vector3d posInCamera = worldToCamera * posInWorld;
  return _cameraModel.getImgFromObject(eigen2CV(posInCamera));
}

cv::Point2f CameraState::imgFromFieldPosition(const Eigen::Vector3d& pos_in_field) const
{
  if (!has_camera_field_transform)
  {
    throw std::runtime_error(DEBUG_INFO + "no camera_field_transform available");
  }
  Eigen::Vector3d point_in_camera = camera_from_field * pos_in_field;
  return getCameraModel().getImgFromObject(eigen2CV(point_in_camera));
}

PanTilt CameraState::panTiltFromXY(const cv::Point2f& pos, double height)
{
  return PanTilt(Eigen::Vector3d(pos.x, pos.y, -height));
}

double CameraState::computeBallRadiusFromPixel(const cv::Point2f& ballPosImg) const
{
  Ray viewRay = getRayInWorldFromPixel(ballPosImg);
  if (viewRay.dir.z() >= 0)
  {
    return -1;
  }

  Plane ballPlane(Eigen::Vector3d::UnitZ(), Constants::field.ball_radius);

  if (!isIntersectionPoint(viewRay, ballPlane))
  {
    return -1;
  }

  Eigen::Vector3d ballCenter = getIntersection(viewRay, ballPlane);

  // Getting a perpendicular direction. We know that viewRay.dir.z<0, thus the
  // vectors will be different
  Eigen::Vector3d groundDir = viewRay.dir;
  groundDir(2) = 0;
  Eigen::Vector3d altDir = viewRay.dir.cross(groundDir).normalized();

  // Getting pixel for ballSide
  double side_sum = 0;
  int nb_points = 0;
  // Testing two directions because one can be out of the image
  for (int side : { -1, 1 })
  {
    // This is not an exact method, but the approximation should be good enough
    Eigen::Vector3d ballSide = ballCenter + side * altDir * Constants::field.ball_radius;
    try
    {
      cv::Point ballSideImg = imgXYFromWorldPosition(ballSide);
      side_sum += (cv2Eigen(ballPosImg) - cv2Eigen(ballSideImg)).norm();
      nb_points++;
    }
    catch (const std::runtime_error& exc)
    {
    }
  }
  if (nb_points == 0)
    return -1;
  return side_sum / nb_points;
}

Eigen::Vector3d CameraState::ballInWorldFromPixel(const cv::Point2f& pos) const
{
  return posInWorldFromPixel(pos, Constants::field.ball_radius);
}

rhoban_geometry::Ray CameraState::getRayInWorldFromPixel(const cv::Point2f& img_pos) const
{
  Eigen::Vector3d viewVectorInCamera = cv2Eigen(_cameraModel.getViewVectorFromImg(img_pos));
  Eigen::Vector3d viewVectorInWorld = cameraToWorld.linear() * viewVectorInCamera;

  Eigen::Vector3d cameraPosInWorld = cameraToWorld * Eigen::Vector3d::Zero();

  return Ray(cameraPosInWorld, viewVectorInWorld);
}

Eigen::Vector3d CameraState::posInWorldFromPixel(const cv::Point2f& pos, double ground_height) const
{
  Ray viewRay = getRayInWorldFromPixel(pos);
  Plane groundPlane(Eigen::Vector3d(0, 0, 1), ground_height);

  if (!isIntersectionPoint(viewRay, groundPlane))
  {
    std::ostringstream oss;
    oss << DEBUG_INFO << " Point " << pos.x << " " << pos.y << " does not intersect ground" << std::endl;
    throw std::runtime_error(oss.str());
  }

  return getIntersection(viewRay, groundPlane);
}

::rhoban_utils::TimeStamp CameraState::getTimeStamp() const
{
  return ::rhoban_utils::TimeStamp::fromMS(monotonic_ts / 1000);
}

uint64_t CameraState::getTimeStampUs() const
{
  return (uint64_t)(monotonic_ts * std::pow(10, 6));
}

double CameraState::getTimeStampDouble() const
{
  return monotonic_ts * 1000;
}

void CameraState::setClockOffset(int64_t new_offset)
{
  clock_offset = new_offset;
}

}  // namespace Utils
}  // namespace Vision
