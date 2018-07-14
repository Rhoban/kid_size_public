#include "CameraState.hpp"
#include <iostream>
#include "Utils/HomogeneousTransform.hpp"
#include "services/ModelService.h"

#include <rhoban_utils/util.h>
#include <rhoban_utils/logging/logger.h>

#include <cmath>

#include <Eigen/StdVector>
#include <utility>
#include <vector>
#include <sstream>
#include <algorithm>

#include <robocup_referee/constants.h>

#include <string>

using namespace rhoban_utils;
using namespace robocup_referee;

static rhoban_utils::Logger logger("CameraState");

namespace Vision {
namespace Utils {

CameraState::CameraState(MoveScheduler *moveScheduler) :
  _pastReadModel(InitHumanoidModel<Leph::HumanoidFixedPressureModel>())
{
  _moveScheduler = moveScheduler;
  _cameraModel = _moveScheduler->getServices()->model->getCameraModel();
  double now = ::rhoban_utils::TimeStamp::now().getTimeMS() / 1000.0;
  _moveScheduler->getServices()->model->pastReadModel(now, _pastReadModel);
  _model = &(_pastReadModel.get());
  _model->setAutoUpdate(false);
  _model->updateDOFPosition();
}

Leph::HumanoidModel & CameraState::getHumanoidModel() const
{
  return *_model;
}

const Leph::CameraModel & CameraState::getCameraModel() const
{
  return _cameraModel;
}

void CameraState::updateInternalModel(double timeStamp) {
  _timeStamp = timeStamp;

  _cameraModel = _moveScheduler->getServices()->model->getCameraModel();
  _model->setAutoUpdate(true);
  _moveScheduler->getServices()->model->pastReadModel(timeStamp,
                                                      _pastReadModel);
  _model = &(_pastReadModel.get());
  _model->setAutoUpdate(false);
  _model->updateDOFPosition();
}


Angle CameraState::getTrunkYawInWorld()
{
  return Angle(rad2deg(_model->orientationYaw("trunk", "origin")));
}

cv::Point2f CameraState::robotPosFromImg(double imgX, double imgY) const
{
  cv::Point2f posInWorldCV = worldPosFromImg(imgX, imgY);

  Eigen::Vector3d posInWorld(posInWorldCV.x, posInWorldCV.y, 0);
  Eigen::Vector3d posInSelf = _model->frameInSelf("origin", posInWorld);

  return cv::Point2f(posInSelf(0), posInSelf(1));
}

cv::Point2f CameraState::worldPosFromImg(double imgX, double imgY) const{

  Eigen::Vector2d pixel(imgX, imgY);

  Eigen::Vector3d viewVector = _model->cameraPixelToViewVector(_cameraModel, pixel);
  Eigen::Vector3d posInWorld;


  bool success = _model->cameraViewVectorToWorld(viewVector, posInWorld);
  if (!success) {
    std::ostringstream oss;
    oss << DEBUG_INFO << "Warning asking for point above horizon ! " << imgX << " " << imgY
        << std::endl;
    throw std::runtime_error(oss.str());
  }

  return cv::Point2f(posInWorld(0), posInWorld(1));
}

Eigen::Vector2d CameraState::getVecInSelf(const Eigen::Vector2d & vec_in_world) const
{
  Eigen::Vector3d src_in_world = Eigen::Vector3d::Zero();
  Eigen::Vector3d dst_in_world = Eigen::Vector3d::Zero();
  dst_in_world.segment(0,2) = vec_in_world;

  Eigen::Vector3d src_in_self, dst_in_self;
  src_in_self = _model->frameInSelf("origin", src_in_world);
  dst_in_self = _model->frameInSelf("origin", dst_in_world);

  return (dst_in_self - src_in_self).segment(0,2);
}

cv::Point2f CameraState::getPosInSelf(const cv::Point2f & pos_in_origin) const
{
  Eigen::Vector3d pos_in_origin_3d(pos_in_origin.x, pos_in_origin.y, 0);
  Eigen::Vector3d pos_in_self_3d;
  pos_in_self_3d = _model->frameInSelf("origin", pos_in_origin_3d);
  return cv::Point2f(pos_in_self_3d(0), pos_in_self_3d(1));
}

std::pair<Angle, Angle> CameraState::robotPanTiltFromImg(double imgX, double imgY) const
{
  Eigen::Vector2d pixel(imgX, imgY);
  Eigen::Vector2d panTilt = _model->cameraPixelToPanTilt(_cameraModel, pixel);
  std::pair<Angle, Angle> result(rad2deg(panTilt(0)), rad2deg(panTilt(1)));
  return result;
}

Angle CameraState::getPitch() {
  return robotPanTiltFromImg(0,0).second;
}

Angle CameraState::getYaw() {
  return robotPanTiltFromImg(0,0).first;
}

double CameraState::getHeight() {
  Eigen::Vector3d pos = Eigen::Vector3d::Zero();

  // Changing the frame from camera to self (robot).
  pos = _model->frameInSelf("camera", pos);
  double height = pos(2);
  if (height < 0) {
    height = 0;
  }
  return height;
}

cv::Point CameraState::imgXYFromRobotPosition(const cv::Point2f &p) const
{
  Eigen::Vector3d posInSelf(p.x, p.y, 0);
  Eigen::Vector3d posInWorld = _model->selfInFrame("origin", posInSelf);
  return imgXYFromWorldPosition(cv::Point(posInWorld(0), posInWorld(1)));
}

cv::Point CameraState::imgXYFromWorldPosition(const cv::Point2f &p) const
{
  Eigen::Vector3d pos(p.x, p.y, 0);
  Eigen::Vector2d pixel;
  bool success = _model->cameraWorldToPixel(_cameraModel, pos, pixel);

  /// Specific case: point was behind the camera -> impossible to have a position
  if (!success && pixel.x() == 0 && pixel.y() == 0) {
    throw std::runtime_error(DEBUG_INFO + " failed to get XY from World Position");
  }

  return cv::Point(pixel.x(), pixel.y());
}

//Should this really be static?? Cannot distort the pixel here...
cv::Point2f CameraState::xyFromPanTilt(const Angle &pan, const Angle &tilt,
                                       double height) {
  double dist = tan(Angle(90) - tilt) * height;
  return cv::Point2f(dist * cos(pan), dist * sin(pan));
}

std::pair<Angle, Angle> CameraState::panTiltFromXY(const cv::Point2f &pos,
                                                   double height) {
  std::pair<Angle, Angle> result;
  double hDist = std::sqrt(pos.x * pos.x + pos.y * pos.y);
  result.first = Angle::fromXY(pos.x, pos.y);
  result.second = Angle::fromXY(hDist, height);
  return result;
}

Eigen::Vector3d CameraState::ballInfoFromPixel(const cv::Point2f &pos,
                                               int *radiusMin, int *radiusMax,
                                               double angularPitchError) const {
  double radiusMinDouble, radiusMaxDouble;
  Eigen::Vector3d posInWorld =
    ballInfoFromPixel(pos,&radiusMinDouble,&radiusMaxDouble, angularPitchError);
  if (radiusMin != nullptr) {
    *radiusMin = std::round(radiusMinDouble);
  }
  if (radiusMax != nullptr) {
    *radiusMax = std::round(radiusMaxDouble);
  }
  return posInWorld;
}

Eigen::Vector3d CameraState::ballInfoFromPixel(const cv::Point2f &pos,
                                               double *radiusMin, double *radiusMax,
                                               double angularPitchError) const {
  if (angularPitchError < 0) {
    angularPitchError = _angularPitchErrorDefault;
  }
  angularPitchError = deg2rad(angularPitchError);


  Eigen::Vector2d pixel = cv2Eigen(pos);
  
  Eigen::Vector2d panTilt = _model->cameraPixelToPanTilt(_cameraModel, pixel);

  Eigen::Vector3d viewVectorUp = _model->cameraPanTiltToViewVector(
    panTilt + Eigen::Vector2d(0.0, angularPitchError));
  Eigen::Vector3d viewVectorMiddle = _model->cameraPanTiltToViewVector(panTilt);
  Eigen::Vector3d viewVectorDown = _model->cameraPanTiltToViewVector(
    panTilt + Eigen::Vector2d(0.0, -angularPitchError));

  // Up
  Eigen::Vector3d ballCenterUp;
  Eigen::Vector2d ballCenterPixelUp;
  std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> bordersPixelUp;
  _model->cameraViewVectorToBallWorld(_cameraModel, viewVectorUp, Constants::field.ballRadius,
                                      ballCenterUp, &ballCenterPixelUp,
                                      &bordersPixelUp);
  // Middle
  Eigen::Vector3d ballCenterMiddle;
  Eigen::Vector2d ballCenterPixelMiddle;
  std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> bordersPixelMiddle;
  _model->cameraViewVectorToBallWorld(
    _cameraModel, viewVectorMiddle, Constants::field.ballRadius, ballCenterMiddle,
    &ballCenterPixelMiddle, &bordersPixelMiddle);
  // Down
  Eigen::Vector3d ballCenterDown;
  Eigen::Vector2d ballCenterPixelDown;
  std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> bordersPixelDown;
  _model->cameraViewVectorToBallWorld(_cameraModel, viewVectorDown,
                                      Constants::field.ballRadius, ballCenterDown,
                                      &ballCenterPixelDown, &bordersPixelDown);

  // Compute minimum and maximum radius
  double minUp = -1.0;
  double minMiddle = -1.0;
  double minDown = -1.0;
  for (size_t i = 0; i < bordersPixelMiddle.size(); i++) {
    double rUp = (ballCenterPixelUp - bordersPixelUp[i]).norm();
    double rMiddle = (ballCenterPixelMiddle - bordersPixelMiddle[i]).norm();
    double rDown = (ballCenterPixelDown - bordersPixelDown[i]).norm();
    if (minUp < 0.0 || minUp > rUp)
      minUp = rUp;
    if (minMiddle < 0.0 || minMiddle > rMiddle)
      minMiddle = rMiddle;
    if (minDown < 0.0 || minDown > rDown)
      minDown = rDown;
  }

  if (minUp < 0.0)
    minUp = 0.0;
  if (minMiddle < 0.0)
    minMiddle = 0.0;
  if (minDown < 0.0)
    minDown = 0.0;

  if (radiusMin != nullptr) {
    *radiusMin = std::min(std::min(minUp, minDown), minMiddle);
  }
  if (radiusMax != nullptr) {
    *radiusMax = std::max(std::max(minUp, minDown), minMiddle);
  }

  return ballCenterMiddle;
}
Eigen::Vector3d CameraState::ballInfoFromPixel(const cv::Point2f &pos) const {
  Eigen::Vector2d pixel = cv2Eigen(pos);

  Eigen::Vector3d ballCenter;
  _model->cameraViewVectorToBallWorld(
      _cameraModel, _model->cameraPixelToViewVector(_cameraModel, pixel),
      Constants::field.ballRadius, ballCenter);

  return ballCenter;
}

::rhoban_utils::TimeStamp CameraState::getTimeStamp() const {
  return ::rhoban_utils::TimeStamp::fromMS(_timeStamp * 1000);
}

double CameraState::getTimeStampDouble() const {
  return _timeStamp * 1000;
}

double CameraState::getPixelYtAtHorizon(double pixelX)
{
  return _model->cameraScreenHorizon(_cameraModel, pixelX);
}

}
}
