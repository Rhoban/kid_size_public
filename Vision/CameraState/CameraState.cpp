#include "CameraState.hpp"
#include <iostream>

#include "camera_state.pb.h"

#include "Utils/HomogeneousTransform.hpp"
#include "services/ModelService.h"
#include "services/ViveService.h"

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

using namespace rhoban_geometry;
using namespace rhoban_utils;
using namespace robocup_referee;

static rhoban_utils::Logger logger("CameraState");

namespace Vision
{
namespace Utils
{
Eigen::Affine3d getAffineFromProtobuf(const rhoban_vision_proto::Pose3D& pose)
{
  Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
  Eigen::Vector3d translation = Eigen::Vector3d::Zero();
  switch (pose.rotation_size())
  {
    case 0:
      break;
    case 4:
    {
      Eigen::Quaterniond q(pose.rotation(0), pose.rotation(1), pose.rotation(2), pose.rotation(3));
      rotation = Eigen::Matrix3d(q);
      break;
    }
    default:
      throw std::runtime_error(DEBUG_INFO + " invalid size for rotation: " + std::to_string(pose.rotation_size()));
  }
  switch (pose.translation_size())
  {
    case 0:
      break;
    case 3:
      for (int dim = 0; dim < 3; dim++)
      {
        translation(dim) = pose.translation(dim);
      }
      break;
    default:
      throw std::runtime_error(DEBUG_INFO +
                               " invalid size for translation: " + std::to_string(pose.translation_size()));
  }
  return Eigen::Translation3d(translation) * Eigen::Affine3d(rotation);
}

void setProtobufFromAffine(const Eigen::Affine3d& affine, rhoban_vision_proto::Pose3D* pose)
{
  pose->clear_rotation();
  pose->clear_translation();
  Eigen::Quaterniond q(affine.linear());
  pose->add_rotation(q.w());
  pose->add_rotation(q.x());
  pose->add_rotation(q.y());
  pose->add_rotation(q.z());
  pose->add_translation(affine.translation()(0));
  pose->add_translation(affine.translation()(1));
  pose->add_translation(affine.translation()(2));
}

CameraState::CameraState(MoveScheduler* moveScheduler)
  : _pastReadModel(InitHumanoidModel<Leph::HumanoidFixedPressureModel>())
{
  _moveScheduler = moveScheduler;
  _cameraModel = _moveScheduler->getServices()->model->getCameraModel();
  double now = ::rhoban_utils::TimeStamp::now().getTimeMS() / 1000.0;
  _moveScheduler->getServices()->model->pastReadModel(now, _pastReadModel);
  _model = &(_pastReadModel.get());
  _model->setAutoUpdate(false);
  _model->updateDOFPosition();
}

CameraState::CameraState(const rhoban_vision_proto::IntrinsicParameters& camera_parameters,
                         const rhoban_vision_proto::CameraState& cs)
  : _moveScheduler(nullptr), _pastReadModel(InitHumanoidModel<Leph::HumanoidFixedPressureModel>())
{
  importFromProtobuf(camera_parameters);
  importFromProtobuf(cs);
}

void CameraState::importFromProtobuf(const rhoban_vision_proto::IntrinsicParameters& camera_parameters)
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

void CameraState::importFromProtobuf(const rhoban_vision_proto::CameraState& src)
{
  _timeStamp = src.time_stamp();
  cameraToWorld = getAffineFromProtobuf(src.camera_to_world());
  selfToWorld = getAffineFromProtobuf(src.self_to_world());
  worldToCamera = cameraToWorld.inverse();
  worldToSelf = selfToWorld.inverse();
}

void CameraState::exportToProtobuf(rhoban_vision_proto::IntrinsicParameters* dst) const
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

void CameraState::exportToProtobuf(rhoban_vision_proto::CameraState* dst) const
{
  dst->set_time_stamp(_timeStamp);
  setProtobufFromAffine(cameraToWorld, dst->mutable_camera_to_world());
  setProtobufFromAffine(selfToWorld, dst->mutable_self_to_world());
}

const Leph::CameraModel& CameraState::getCameraModel() const

{
  return _cameraModel;
}

void CameraState::updateInternalModel(double timeStamp)
{
  _timeStamp = timeStamp;

  if (_moveScheduler != nullptr)
  {
    ViveService * vive = _moveScheduler->getServices()->vive;
    if (vive->isActive())
    {
      worldToCamera = vive->getFieldToVive((int64)(timeStamp * 1000 * 1000));
      //TODO: update worldToSelf properly (require to use head info as well)
      worldToSelf = Eigen::Affine3d();
    }
    else
    {
      _model->setAutoUpdate(true);
      _moveScheduler->getServices()->model->pastReadModel(timeStamp, _pastReadModel);
      _model = &(_pastReadModel.get());
      _model->setAutoUpdate(false);
      _model->updateDOFPosition();
    
      worldToSelf = _model->selfFrameTransform("origin");
      worldToCamera = _model->getTransform("camera", "origin");
    }
    _cameraModel = _moveScheduler->getServices()->model->getCameraModel();
    selfToWorld = worldToSelf.inverse();
    cameraToWorld = worldToCamera.inverse();
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
  Eigen::Vector3d pos_in_self = selfToWorld * Eigen::Vector3d(pos_in_origin.x, pos_in_origin.y, 0);
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
  PanTilt panTilt(cameraToWorld.linear() * Eigen::Vector3d::UnitZ());
  return panTilt.tilt;
}

Angle CameraState::getYaw()
{
  PanTilt panTilt(cameraToWorld.linear() * Eigen::Vector3d::UnitZ());
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

cv::Point CameraState::imgXYFromWorldPosition(const cv::Point2f& p) const
{
  return imgXYFromWorldPosition(Eigen::Vector3d(p.x, p.y, 0));
}

cv::Point CameraState::imgXYFromWorldPosition(const Eigen::Vector3d& posInWorld) const
{
  Eigen::Vector3d posInCamera = worldToCamera * posInWorld;
  return _cameraModel.getImgFromObject(eigen2CV(posInCamera));
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

  Plane ballPlane(Eigen::Vector3d::UnitZ(), Constants::field.ballRadius);

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

  // Getting one of the points on the side of the ball, this is not an exact
  // method, but the approximation should be good enough
  Eigen::Vector3d ballSide = ballCenter + altDir * Constants::field.ballRadius;

  // Getting pixel for ballSide
  cv::Point ballSideImg = imgXYFromWorldPosition(ballSide);

  return (cv2Eigen(ballPosImg) - cv2Eigen(ballSideImg)).norm();
}

Eigen::Vector3d CameraState::ballInWorldFromPixel(const cv::Point2f& pos) const
{
  return posInWorldFromPixel(pos, Constants::field.ballRadius);
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
  return ::rhoban_utils::TimeStamp::fromMS(_timeStamp * 1000);
}

double CameraState::getTimeStampDouble() const
{
  return _timeStamp * 1000;
}

}  // namespace Utils
}  // namespace Vision
