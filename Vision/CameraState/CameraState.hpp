#pragma once

#include "scheduler/MoveScheduler.h"

#include <hl_communication/camera.pb.h>
#include <opencv2/core/core.hpp>
#include <rhoban_geometry/3d/ray.h>
#include <rhoban_geometry/3d/pan_tilt.h>
#include <rhoban_utils/angle.h>
#include <rhoban_utils/timing/time_stamp.h>
#include <robot_model/camera_model.h>

#include <utility>
#include <string>
#include <stdexcept>

namespace Vision
{
namespace Utils
{
/// Relevant basis:
/// - World: fixed reference in which the camera is evolving
/// - Self: A basis centered on the robot
///   - Center: Projection of trunk of the robot on the ground
///   - X-axis: in front of the robot
///   - Y-axis: left of the robot
///   - Z-axis: same as world axis
/// - Camera:
///   - Center: At camera optical center
///   - X-axis: aligned with x-axis of the image
///   - Y-axis: aligned with y-axis of the image
///   - Z-axis: direction toward which the camera is pointing
class CameraState
{
public:
  CameraState();
  CameraState(MoveScheduler* moveScheduler);
  CameraState(const hl_communication::IntrinsicParameters& camera_parameters,
              const hl_communication::FrameEntry& frame_entry, const hl_communication::Pose3D& camera_from_self,
              const hl_communication::VideoSourceID& source_id);

  cv::Size getImgSize() const;

  void importFromProtobuf(const hl_communication::IntrinsicParameters& camera_parameters);
  void importFromProtobuf(const hl_communication::FrameEntry& src);
  void exportToProtobuf(hl_communication::IntrinsicParameters* dst) const;
  void exportToProtobuf(hl_communication::FrameEntry* dst) const;
  /**
   * Import source_id and camera_parameters from the VideoMetaInformation
   */
  void importHeader(const hl_communication::VideoMetaInformation& src);
  /**
   * Export source_id and camera_parameters to the VideoMetaInformation
   */
  void exportHeader(hl_communication::VideoMetaInformation* dst) const;

  const rhoban::CameraModel& getCameraModel() const;

  /// Asks the model to update itself to the state the robot had at given timestamps
  /// Both monotonic and utc_ts should be provided
  void updateInternalModel(const rhoban_utils::TimeStamp& ts);

  /// Return the [x,y] position of the ground point seen at (imgX, imgY)
  /// in self referential [m]
  /// throws a runtime_error if the point requested is above horizon
  cv::Point2f robotPosFromImg(double imgX, double imgY) const;

  /// Return the [x,y] position of the ground point seen at (imgX, imgY)
  /// in world referential [m]
  /// throws a runtime_error if the point requested is above horizon
  cv::Point2f worldPosFromImg(double imgX, double imgY) const;

  /// Converting vector from world referential to self referential
  Eigen::Vector2d getVecInSelf(const Eigen::Vector2d& vec_in_world) const;

  /// Return the position in the robot basis from a position in origin basis
  cv::Point2f getPosInSelf(const cv::Point2f& pos_in_origin) const;

  /// Return the [pan, tilt] pair of the ground point seen at imgX, imgY
  rhoban_geometry::PanTilt robotPanTiltFromImg(double imgX, double imgY) const;

  /// Convert the position 'pos_camera' (in camera referential) to the 'world' basis
  Eigen::Vector3d getWorldPosFromCamera(const Eigen::Vector3d& pos_camera) const;

  /// Convert the position 'pos_world' (in world referential) to the 'self' basis
  Eigen::Vector3d getSelfFromWorld(const Eigen::Vector3d& pos_world) const;

  /// Convert the position 'pos_self' (in self referential) to the 'world' basis
  Eigen::Vector3d getWorldFromSelf(const Eigen::Vector3d& pos_self) const;

  /*
   * Returns the xy position expected on the screen of the point p [m]
   * throws a std::runtime_error if point is behind the camera
   */
  cv::Point2f imgXYFromWorldPosition(const cv::Point2f& p) const;
  cv::Point2f imgXYFromWorldPosition(const Eigen::Vector3d& p) const;

  /**
   * Returns position of the point from its field position.
   * throws a std::runtime_error if camera_field_transform is not available or if point is outside of the image.
   */
  cv::Point2f imgFromFieldPosition(const Eigen::Vector3d& p) const;

  /**
   * Return the pan,tilt position respectively on the robot basis, from xy in
   * robot basis.
   */
  static rhoban_geometry::PanTilt panTiltFromXY(const cv::Point2f& pos, double height);

  /**
   * Compute with the model the cartesian position of the
   * ball in model world frame viewed in the image at given
   * pixel.
   *
   * throw a runtime_error if corresponding ray does not intersect ball plane
   */
  Eigen::Vector3d ballInWorldFromPixel(const cv::Point2f& img_pos) const;

  /**
   * Return the ray starting at camera source and going toward direction of img_pos
   */
  rhoban_geometry::Ray getRayInWorldFromPixel(const cv::Point2f& img_pos) const;

  /// Get the intersection between the ray corresponding to the pixel 'img_pos'
  /// and the horizontal plane at plane_height
  /// throw a runtime_error if corresponding ray does not intersect with the plane
  Eigen::Vector3d posInWorldFromPixel(const cv::Point2f& img_pos, double plane_height = 0) const;

  /**
   * Return the expected radius for a ball at the given pixel.
   *
   * If the pixel is above horizon, a negative value is returned
   *
   * Note: this method is an approximation, the exact method could have 4
   * different results which are the intersection of a plane and a cone.
   * - Circle
   * - Ellipse
   * - Parabole
   * - Hyperbole
   */
  double computeBallRadiusFromPixel(const cv::Point2f& pos) const;

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
  /**
   * Return the timestamp in micro-seconds since epoch
   */
  uint64_t getTimeStampUs() const;
  /// Return the timestamp in [ms]
  double getTimeStampDouble() const;

  /**
   * Return the timestamp for the scheduler [s] including motor_delay
   */
  double getSchedulerTS() const;
  double getSchedulerTS(const rhoban_utils::TimeStamp& ts) const;

  /**
   * Sets the offset in micro-seconds between
   */
  void setClockOffset(int64_t new_offset);

  MoveScheduler* _moveScheduler;
  rhoban::CameraModel _cameraModel;
  double monotonic_ts;
  uint64_t utc_ts;

  Eigen::Affine3d worldToSelf;
  Eigen::Affine3d selfToWorld;
  Eigen::Affine3d worldToCamera;
  Eigen::Affine3d cameraToWorld;
  Eigen::Affine3d camera_from_head_base;

  /**
   * Depending on information source, transform between camera and field basis is not available
   */
  bool has_camera_field_transform;
  Eigen::Affine3d camera_from_field;
  Eigen::Affine3d field_from_camera;

  /**
   * Positions of the ball in field referential according to Vive
   */
  std::vector<Eigen::Vector3d> vive_balls_in_field;

  /**
   * Positions of the trackers (robots) in field referential according to Vive
   */
  std::vector<Eigen::Vector3d> vive_trackers_in_field;

  /**
   * Offset between steady_clock and system clock for the given camera state
   */
  int64_t clock_offset;

  hl_communication::FrameStatus frame_status;

  /**
   * Identifier of the video source which took the image
   */
  hl_communication::VideoSourceID source_id;

  /**
   * Currently, the motors are not properly timestamped, this allows to retrieve
   * information at the appropriate time for vision
   * unit is [ms]
   */
  static float motor_delay;
};
}  // namespace Utils
}  // namespace Vision
