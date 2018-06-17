#include "CameraState.hpp"
#include <iostream>
#include "Utils/HomogeneousTransform.hpp"
#include "services/ModelService.h"

#include <rhoban_utils/logging/logger.h>

#include <cmath>

#include <Eigen/StdVector>
#include <utility>
#include <vector>
#include <sstream>
#include <algorithm>

#include <robocup_referee/constants.h>

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp> //undistortPoints
#include <string>

#ifndef DEG2RAD
#define DEG2RAD(x) (((x) / 180.0) * M_PI)
#endif
#ifndef RAD2DEG
#define RAD2DEG(x) (((x) / M_PI) * 180.0)
#endif

using namespace rhoban_utils;
using namespace robocup_referee;

static rhoban_utils::Logger logger("CameraState");

namespace Vision {
namespace Utils {

/*
 *TODO :
 * - All the calculations are now done outside the vision, through a model's
 *instance
 * - The workflow should be :
 *     1) Once we know the timestamp of the image, we ask for a model's update
 *on that TS
 * (this could be done in a non-blocking fashion but all methods would then need
 *to check
 * the model's readyness. We could save a few ms like that though)
 *     2) All the methods here will ask the updated model for an answer
 * - Common methods should stay alive to reduce the price of the modifications
 */
CameraState::CameraState(MoveScheduler *moveScheduler,const std::string& CameraParametersYAML) :
  _pastReadModel(InitHumanoidModel<Leph::HumanoidFixedPressureModel>())
{
  _moveScheduler = moveScheduler;
  _params = _moveScheduler->getServices()->model->getCameraParameters();
  double now = ::rhoban_utils::TimeStamp::now().getTimeMS() / 1000.0;
  _moveScheduler->getServices()->model->pastReadModel(now, _pastReadModel);
  _model = &(_pastReadModel.get());
  _model->setAutoUpdate(false);
  _model->updateDOFPosition();

  //TODO Now, camera parameters are loaded from Vision Camera Model (VCM.json) in ModelService
  //Aruco camera model is disabled
  /*
  try {
    std::cout << "Reading camera parameters from " << CameraParametersYAML << std::endl;
    CamParam.readFromXMLFile(CameraParametersYAML);

    //Don't worry, these are not important to compute fovx and fovy. But for the other paramters, it is.
    //FIXME: add a clean way to integrate these camera specific values
    double appertureW=1280*0.00375;
    double appertureH=960*0.00375;

    double focalLength,aspectRatio;
    cv::Point2d principalPoint;
    cv::calibrationMatrixValues(CamParam.CameraMatrix,cv::Size(CamParam.CamSize.width,CamParam.CamSize.height),appertureW,appertureH,fovx,fovy,focalLength,principalPoint,aspectRatio);
    //Overriding the FOV parameters in the model...
    moveScheduler->getServices()->model->setCameraParametersFOV(DEG2RAD(fovx),DEG2RAD(fovy));

  } catch (...) {
    throw std::runtime_error("ERROR! You initialized a CameraState without the camera parameters (yaml file)");
  }
  */

}

void CameraState::updateInternalModel(double timeStamp) {
  _timeStamp = timeStamp;

  _params = _moveScheduler->getServices()->model->getCameraParameters();
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

cv::Point2f CameraState::robotPosFromImg(double imgX, double imgY,
                                         double imgWidth, double imgHeight,
                                         bool self) {
  cv::Point2f undist;
  undistortPoint(imgX, imgY, imgWidth, imgHeight, undist);

// x and y are in [-1, 1] of the screen
  double x = (undist.x - imgWidth / 2) * 2 / imgWidth;
  double y = (undist.y - imgHeight / 2) * 2 / imgHeight;

  Eigen::Vector2d pixel(x, y);
  Eigen::Vector3d pos;


  bool success = _model->cameraViewVectorToWorld(
    _model->cameraPixelToViewVector(_params, pixel), pos);
  if (!success) {
    std::ostringstream oss;
    oss << "Warning asking for point above horizon ! " << x << " " << y
        << std::endl;
    throw std::runtime_error(oss.str());
  }
  if (self) {
    // Changing the frame from origin (of world) to self (robot)
    pos = _model->frameInSelf("origin", pos);
  }

  return cv::Point2f(pos(0), pos(1));
}

Eigen::Vector2d CameraState::getVecInSelf(const Eigen::Vector2d & vec_in_world)
{
  Eigen::Vector3d src_in_world = Eigen::Vector3d::Zero();
  Eigen::Vector3d dst_in_world = Eigen::Vector3d::Zero();
  dst_in_world.segment(0,2) = vec_in_world;

  Eigen::Vector3d src_in_self, dst_in_self;
  src_in_self = _model->frameInSelf("origin", src_in_world);
  dst_in_self = _model->frameInSelf("origin", dst_in_world);

  return (dst_in_self - src_in_self).segment(0,2);
}

cv::Point2f CameraState::getPosInSelf(const cv::Point2f & pos_in_origin)
{
  Eigen::Vector3d pos_in_origin_3d(pos_in_origin.x, pos_in_origin.y, 0);
  Eigen::Vector3d pos_in_self_3d;
  pos_in_self_3d = _model->frameInSelf("origin", pos_in_origin_3d);
  return cv::Point2f(pos_in_self_3d(0), pos_in_self_3d(1));
}

/**
 * Return the [pan, tilt] pair of the ground point seen at imgX, imgY
 */
std::pair<Angle, Angle> CameraState::robotPanTiltFromImg(double imgX,
                                                         double imgY,
                                                         double imgWidth,
                                                         double imgHeight) {


  //imgX ang imgY are in [0,1]
  cv::Point2f undist;
  undistortPoint(imgX, imgY, imgWidth, imgHeight, undist);

// x and y are in [-1, 1] of the screen
  double x = (undist.x - imgWidth / 2) * 2 / imgWidth;
  double y = (undist.y - imgHeight / 2) * 2 / imgHeight;
  Eigen::Vector2d pixel(x, y);
  Eigen::Vector2d panTilt;

  // This function can return the direction vector if asked kindly
  panTilt = _model->cameraPixelToPanTilt(_params, pixel);
  std::pair<Angle, Angle> result(RAD2DEG(panTilt(0)), RAD2DEG(panTilt(1)));

  // printf("KKK: x=%g, y=%g, W=%g, H=%g, Pixel %g %g, pan=%g, tilt=%g\n",
  //     imgX, imgY, imgWidth, imgHeight, x, y, result.first.getSignedValue(),
  //     result.second.getSignedValue());

  return result;
}

Angle CameraState::getPitch() {
  // Asking pan and tilt of the center of the image...
  Eigen::Vector2d pixel(0, 0);
  Eigen::Vector2d panTilt;

  panTilt = _model->cameraPixelToPanTilt(_params, pixel);

  return Angle(RAD2DEG(panTilt(1)));
}

Angle CameraState::getYaw() {
  // Asking pan and tilt of the center of the image...
  Eigen::Vector2d pixel(0, 0);
  Eigen::Vector2d panTilt;

  panTilt = _model->cameraPixelToPanTilt(_params, pixel);

  return Angle(RAD2DEG(panTilt(0)));
}

double CameraState::getLatApertureDeg() {
  return rad2deg(_params.widthAperture);
}

double CameraState::getVertApertureDeg() {
  return rad2deg(_params.heightAperture);
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

/*
void CameraState::normalizePointFromCamMatrix(const Eigen::Vector2d &in, cv::Point3f &out)
{
  //Normalize the pixel from image coord (width,height) to a coordinate usable for projectPoints()
  out = cv::Point3f((in(0) - CamParam.CameraMatrix.at<float>(0, 2)) /
                    CamParam.CameraMatrix.at<float>(0, 0),
                    (in(1) - CamParam.CameraMatrix.at<float>(1, 2)) /
                        CamParam.CameraMatrix.at<float>(1, 1),
                    1);
}


void CameraState::unnormalizePointFromCamMatrix(const cv::Point2d &in, cv::Point2f &out)
{
  //unnormalize the pixel from image coord (width,height) to a coordinate usable for projectPoints()
  out = cv::Point2f((in.x  *  CamParam.CameraMatrix.at<float>(0, 0)+ CamParam.CameraMatrix.at<float>(0, 2)),
                    (in.y  * CamParam.CameraMatrix.at<float>(1, 1)+ CamParam.CameraMatrix.at<float>(1, 2)));
}
*/

void CameraState::distortPoint(double pixelX, double pixelY, double imgWidth, double imgHeight, cv::Point2f &out)
{
  out=cv::Point2f(pixelX,pixelY);

  /* TODO For now, distortion model from ARUCO is disabled
  if(CamParam.CamSize.height != -1) //if the parameters were loaded
  {
    //Convert to the pixel space
    Eigen::Vector2d orig_pixel(pixelX/imgWidth*CamParam.CamSize.width,pixelY/imgHeight*CamParam.CamSize.height);
    //Distort the pixel
    cv::Point3f normalized;
    std::vector<cv::Point2f> distorted;
    normalizePointFromCamMatrix(orig_pixel,normalized);
    std::vector<cv::Point3f> undistorted;
    undistorted.push_back(normalized);
    cv::projectPoints(undistorted,cv::Mat::zeros(3, 1, CV_64FC1), cv::Mat::zeros(3, 1, CV_64FC1),CamParam.CameraMatrix,CamParam.Distorsion,distorted);

    out=cv::Point2f(distorted[0].x/CamParam.CamSize.width*imgWidth,distorted[0].y/CamParam.CamSize.height*imgHeight);
  }
  else
  {
    //Don't do anything
    out=cv::Point2f(pixelX,pixelY);
  }
  */
}

void CameraState::undistortPoint(double pixelX, double pixelY, double imgWidth, double imgHeight, cv::Point2f &out)
{
  out=cv::Point2f(pixelX,pixelY);
  
  /* TODO For now, distortion model from ARUCO is disabled
  if(CamParam.CamSize.height != -1) //if the parameters were loaded
  {

    //convert to the pixel space
    std::vector<cv::Point2d> distortedPoint;
    distortedPoint.push_back(cv::Point2d(pixelX/imgWidth*CamParam.CamSize.width,pixelY/imgHeight*CamParam.CamSize.height));
    std::vector<cv::Point2d> undistortedPoint;

    //Correct the pixel
    cv::undistortPoints(distortedPoint,undistortedPoint,CamParam.CameraMatrix,CamParam.Distorsion);
    cv::Point2f unnorm;
    unnormalizePointFromCamMatrix(undistortedPoint[0], unnorm);

    //get back to the original space
    double undistX=unnorm.x/CamParam.CamSize.width*imgWidth;
    double undistY=unnorm.y/CamParam.CamSize.height*imgHeight;

    out=cv::Point2f(undistX,undistY);
  }
  else
  {
    //Don't do anything
    out=cv::Point2f(pixelX,pixelY);
  }
  */
}

cv::Point CameraState::imgXYFromRobotPosition(const cv::Point2f &p,
                                              double imgWidth, double imgHeight,
                                              bool self) {


  Eigen::Vector2d pixel(0.0, 0.0);
  cv::Point2f distorted(-1.0, -1.0); //In case of false from cameraWorldToPixel
  Eigen::Vector3d pos(p.x, p.y, 0);
  // pos is expressed in the robot's frame, we need it in the world's frame
  if (self) {
    pos = _model->selfInFrame("origin", pos);
  }
  bool success = _model->cameraWorldToPixel(_params, pos, pixel);
  if (!success) {
    return cv::Point(-1,-1);
  } else {
    //pixel is in [-1, 1] => remap in [0, imgHeight] 
    distortPoint((pixel(0)+1.0)/2.0*imgWidth, (pixel(1)+1.0)/2.0* imgHeight, imgWidth, imgHeight, distorted);
  }

  // // Pixel is in [-1, 1]
  // return cv::Point(distorted.x * imgWidth / 2 + imgWidth / 2,
  //                  distorted.y * imgHeight / 2 + imgHeight / 2);

  return cv::Point(distorted.x ,
                   distorted.y );
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
                                               double width, double height,
                                               int *radiusMin, int *radiusMax,
                                               double angularPitchError) {
  if (angularPitchError < 0) {
    angularPitchError = _angularPitchErrorDefault;
  }
  angularPitchError = DEG2RAD(angularPitchError);


  cv::Point2f undistorted;
  undistortPoint(pos.x,pos.y,width,height,undistorted);

  Eigen::Vector2d pixel;
  pixel.x() = ((double)(undistorted.x - width / 2.0) * 2.0 / width);
  pixel.y() = ((double)(undistorted.y - height / 2.0) * 2.0 / height);
  
  Eigen::Vector2d panTilt = _model->cameraPixelToPanTilt(_params, pixel);
  Eigen::Vector3d viewVectorUp = _model->cameraPanTiltToViewVector(
    panTilt + Eigen::Vector2d(0.0, angularPitchError));
  Eigen::Vector3d viewVectorMiddle = _model->cameraPanTiltToViewVector(panTilt);
  Eigen::Vector3d viewVectorDown = _model->cameraPanTiltToViewVector(
    panTilt + Eigen::Vector2d(0.0, -angularPitchError));

  // Up
  Eigen::Vector3d ballCenterUp;
  Eigen::Vector2d ballCenterPixelUp;
  std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> bordersPixelUp;
  _model->cameraViewVectorToBallWorld(_params, viewVectorUp, Constants::field.ballRadius,
                                      ballCenterUp, &ballCenterPixelUp,
                                      &bordersPixelUp);
  // Middle
  Eigen::Vector3d ballCenterMiddle;
  Eigen::Vector2d ballCenterPixelMiddle;
  std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> bordersPixelMiddle;
  _model->cameraViewVectorToBallWorld(
    _params, viewVectorMiddle, Constants::field.ballRadius, ballCenterMiddle,
    &ballCenterPixelMiddle, &bordersPixelMiddle);
  // Down
  Eigen::Vector3d ballCenterDown;
  Eigen::Vector2d ballCenterPixelDown;
  std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> bordersPixelDown;
  _model->cameraViewVectorToBallWorld(_params, viewVectorDown,
                                      Constants::field.ballRadius, ballCenterDown,
                                      &ballCenterPixelDown, &bordersPixelDown);

  //TODO: Pff should we really distort all these?
  // Convert to image space
  ballCenterPixelUp.x() *= width / 2.0;
  ballCenterPixelUp.x() += width / 2.0;
  ballCenterPixelUp.y() *= height / 2.0;
  ballCenterPixelUp.y() += height / 2.0;
  ballCenterPixelMiddle.x() *= width / 2.0;
  ballCenterPixelMiddle.x() += width / 2.0;
  ballCenterPixelMiddle.y() *= height / 2.0;
  ballCenterPixelMiddle.y() += height / 2.0;
  ballCenterPixelDown.x() *= width / 2.0;
  ballCenterPixelDown.x() += width / 2.0;
  ballCenterPixelDown.y() *= height / 2.0;
  ballCenterPixelDown.y() += height / 2.0;

  // Compute minimum and maximum radius
  double minUp = -1.0;
  double minMiddle = -1.0;
  double minDown = -1.0;
  for (size_t i = 0; i < bordersPixelMiddle.size(); i++) {
    bordersPixelUp[i].x() *= width / 2.0;
    bordersPixelUp[i].x() += width / 2.0;
    bordersPixelUp[i].y() *= height / 2.0;
    bordersPixelUp[i].y() += height / 2.0;
    bordersPixelMiddle[i].x() *= width / 2.0;
    bordersPixelMiddle[i].x() += width / 2.0;
    bordersPixelMiddle[i].y() *= height / 2.0;
    bordersPixelMiddle[i].y() += height / 2.0;
    bordersPixelDown[i].x() *= width / 2.0;
    bordersPixelDown[i].x() += width / 2.0;
    bordersPixelDown[i].y() *= height / 2.0;
    bordersPixelDown[i].y() += height / 2.0;
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
    *radiusMin = std::round(std::min(std::min(minUp, minDown), minMiddle));
  }
  if (radiusMax != nullptr) {
    *radiusMax = std::round(std::max(std::max(minUp, minDown), minMiddle));
  }

  return ballCenterMiddle;
}
Eigen::Vector3d CameraState::ballInfoFromPixel(const cv::Point2f &pos,
                                               double width, double height) {
  if (width <= 0 || height <= 0) {
    std::ostringstream oss;
    oss << "Invalid values for (width,height): (" << width << "," << height << ")";
    throw std::logic_error(oss.str());
  }


  cv::Point2f undistorted;
  undistortPoint(pos.x,pos.y,width,height,undistorted);

  Eigen::Vector2d pixel;
  pixel.x() = ((double)(undistorted.x - width / 2.0) * 2.0 / width);
  pixel.y() = ((double)(undistorted.y - height / 2.0) * 2.0 / height);

  Eigen::Vector3d ballCenter;
  _model->cameraViewVectorToBallWorld(
      _params, _model->cameraPixelToViewVector(_params, pixel),
      Constants::field.ballRadius, ballCenter);

  return ballCenter;
}

::rhoban_utils::TimeStamp CameraState::getTimeStamp() {
  return ::rhoban_utils::TimeStamp::fromMS(_timeStamp * 1000);
}

double CameraState::getTimeStampDouble() { return _timeStamp * 1000; }

double CameraState::getPixelYtAtHorizon(double pixelX, double imgWidth,
                                        double imgHeight) {

  //TODO: should we undistort this?
  double x = (pixelX - imgWidth / 2) * 2 / imgWidth;
  double y = _model->cameraScreenHorizon(_params, x);
  return (imgHeight / 2) * y + imgHeight / 2;
}
}
}
