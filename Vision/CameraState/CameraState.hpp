#pragma once

#include "rhoban_utils/angle.h"

#include <opencv2/core/core.hpp>
#include <stdexcept>
#include "scheduler/MoveScheduler.h"
#include <Model/HumanoidFixedPressureModel.hpp>
#include <Model/HumanoidModel.hpp>
#include "rhoban_utils/timing/time_stamp.h"
#include <aruco.h> //access to the camera params struct
#include <utility>
#include <string>

namespace Vision {
namespace Utils {

class CameraState {

public:
  typedef std::pair<rhoban_utils::Angle,rhoban_utils::Angle> PanTilt;
  
  CameraState(MoveScheduler *moveScheduler);

  Leph::HumanoidModel & getHumanoidModel() const;

  const Leph::CameraModel & getCameraModel() const;

  /// Asks the model to update itself to the state the robot had at timeStamp
  void updateInternalModel(double timeStamp);

  /// Return the [x,y] position of the ground point seen at (imgX, imgY)
  /// in self referential [m]
  /// throws a runtime_error if the point requested is above horizon
  cv::Point2f robotPosFromImg(double imgX, double imgY) const;

  /// Return the [x,y] position of the ground point seen at (imgX, imgY)
  /// in world referential [m]
  /// throws a runtime_error if the point requested is above horizon
  cv::Point2f worldPosFromImg(double imgX, double imgY) const;

  /// Converting vector from world referential to self referential
  Eigen::Vector2d getVecInSelf(const Eigen::Vector2d & vec_in_world) const;

  /// Return the position in the robot basis from a position in origin basis
  cv::Point2f getPosInSelf(const cv::Point2f & pos_in_origin) const;

  /// Return the [pan, tilt] pair of the ground point seen at imgX, imgY
  PanTilt robotPanTiltFromImg(double imgX, double imgY) const;

  /*
   * Returns the xy position expected on the screen of the point p [m]
   * Return (-1,-1) if point p is outside of the img
   */
  cv::Point imgXYFromRobotPosition(const cv::Point2f &p) const;
  cv::Point imgXYFromWorldPosition(const cv::Point2f &p) const;

  /**
   * Return the xy position in the robot basis, from pan, tilt respectively to
   * the robot
   * basis.
   */
  static cv::Point2f xyFromPanTilt(const rhoban_utils::Angle &pan,
                                   const rhoban_utils::Angle &tilt,
                                   double height);

  /**
   * Return the pan,tilt position respectively on the robot basis, from xy in
   * robot basis.
   */
  static PanTilt panTiltFromXY(const cv::Point2f &pos, double height);

  /**
   * Compute with the model the cartesian position of the
   * ball in model world frame viewed in the image at given
   * pixel. Also optionally assign minimum and maximum
   * ball radius in pixels.
   * angularPitchError is an angular pitch (tilt) error
   * offset in degrees use to compute min and max ball radius
   * confidence bounds
   * A negative value of angularPitchError means that the value used
   * for angularPitchError is set to angularPitchErrorDefault
   */
  Eigen::Vector3d ballInfoFromPixel(const cv::Point2f &pos) const;
  Eigen::Vector3d ballInfoFromPixel(const cv::Point2f &pos,
                                    int *radiusMin, int *radiusMax,
                                    double angularPitchError = -1.0) const;
  Eigen::Vector3d ballInfoFromPixel(const cv::Point2f &pos,
                                    double *radiusMin, double *radiusMax,
                                    double angularPitchError = -1.0) const;

  /// Distance to ground [m]
  double getHeight();

  /**
   * Pitch in degrees
   *   0 -> looking horizon
   * +90 -> looking the feet
   */
  rhoban_utils::Angle getPitch();

  /**
   * Yaw of the camera basis
   * -X -> right of the robot
   *  0 -> In front of the robot
   * +X -> left of the robot
   */
  rhoban_utils::Angle getYaw();

  /**
   * Yaw of the trunk in the world referential
   *
   */
  rhoban_utils::Angle getTrunkYawInWorld();

  rhoban_utils::TimeStamp getTimeStamp() const;
  /// Return the timestamp in [ms]
  double getTimeStampDouble() const;

  /**
   * Returns the Y coordinate of the pixel on the horizon line whose position in
   * the X coordinate is pixelX
   */
  double getPixelYtAtHorizon(double pixelX);

  MoveScheduler *_moveScheduler;
  Leph::HumanoidFixedPressureModel _pastReadModel;
  Leph::HumanoidModel *_model;
  Leph::CameraModel _cameraModel;
  double _timeStamp;
  double _angularPitchErrorDefault = 0.0;
};
}
}
