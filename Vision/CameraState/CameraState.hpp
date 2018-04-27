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
  
  CameraState(MoveScheduler *moveScheduler, const std::string& CameraParametersYAML="./camera_calib.yml");

  /// Asks the model to update itself to the state the robot had at timeStamp
  void updateInternalModel(double timeStamp);

  /// Return the [x,y] position of the ground point seen at imgX, imgY [0,1]
  /// Return value is in [m]
  /// throws a runtime_error if the point requested is above horizon
  cv::Point2f robotPosFromImg(double imgX, double imgY, double imgWidth,
                              double imgHeight, bool self = true);

  /// Converting vector from world referential to self referential
  Eigen::Vector2d getVecInSelf(const Eigen::Vector2d & vec_in_world);

  /// Return the position in the robot basis from a position in origin basis
  cv::Point2f getPosInSelf(const cv::Point2f & pos_in_origin);

  /// Return the [pan, tilt] pair of the ground point seen at imgX, imgY
  PanTilt robotPanTiltFromImg(double imgX, double imgY,
                              double imgWidth,
                              double imgHeight);

  /*
   * Returns the xy position expected on the screen of the point p
   * expressed in the robot's frame
   * Return (-1,-1) if point p is outside of the img
   */
  cv::Point imgXYFromRobotPosition(const cv::Point2f &p, double imgWidth,
                                   double imgHeight, bool self = true);

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
   * Returns the number of pixels the radius of the ball should take if the ball
   * center was at pos
   */
  //      int ballPixelRadiusFromXY(const cv::Point2f & pos, double height, int
  //      nbCols, int nbRows);

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
  Eigen::Vector3d ballInfoFromPixel(const cv::Point2f &pos, double width,
                                    double height);
  Eigen::Vector3d ballInfoFromPixel(const cv::Point2f &pos, double width,
                                    double height, int *radiusMin, int *radiusMax,
                                    double angularPitchError = -1.0);

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


  /// Return the lateral aperture angle in degrees
  double getLatApertureDeg();

  /// Return the vertical aperture angle in degrees
  double getVertApertureDeg();

  /**
   * Returns the timestamp of the last update in the obnoxius type
   */
  rhoban_utils::TimeStamp getTimeStamp();
  /**
   * Returns the timestamp of the last update in a respectable type
   */
  double getTimeStampDouble();

  /**
   * Returns the Y coordinate of the pixel on the horizon line whose
   * position in the X coordinate is pixelX
   */
  double getPixelYtAtHorizon(double pixelX, double imgWidth, double imgHeight);


  /**
   * Normalize a point from the CameraMatrix. For internal use only...
   */

  void normalizePointFromCamMatrix(const Eigen::Vector2d &in, cv::Point3f &out);

  void unnormalizePointFromCamMatrix(const cv::Point2d &in, cv::Point2f &out);

  /**
   * Undistort points from the real camera to an ideal one.
   */
  void undistortPoint(double pixelX, double pixelY, double imgWidth, double imgHeight, cv::Point2f &out);

  /**
   * Distort points from an ideal camera coordinate to the "real" camera.
   */
  void distortPoint(double pixelX, double pixelY, double imgWidth, double imgHeight, cv::Point2f &out);

  MoveScheduler *_moveScheduler;
  Leph::HumanoidFixedPressureModel _pastReadModel;
  Leph::HumanoidModel *_model;
  Leph::CameraParameters _params;
  double _timeStamp;
  double _angularPitchErrorDefault = 0.0;

  //Parameters that are very confortable here...
  aruco::CameraParameters CamParam;
  double fovx; //Horizontal Field of view in degrees
  double fovy; //Vertical
};
}
}
