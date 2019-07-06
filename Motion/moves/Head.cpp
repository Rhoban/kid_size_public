#include "Head.h"

#include "services/ModelService.h"
#include "services/DecisionService.h"
#include "services/LocalisationService.h"
#include "services/RefereeService.h"

#include "rhoban_utils/logging/logger.h"

#include <rhoban_utils/angle.h>

using namespace rhoban_utils;
using rhoban_unsorted::HeadScan;

static rhoban_utils::Logger logger("Head");

Head::Head()
  : scan_period(-1)
  , pan_deg(0)
  , tilt_deg(0)
  , target_x(0)
  , target_y(0)
  , wished_pan(0)
  , wished_tilt(0)
  , wished_dist(0)
  , is_tracking(false)
  , tracking_time(0)
  , scanning_time(0)
  , last_pan_target(0)
  , last_tilt_target(0)
{
  Move::initializeBinding();
  // Special modes variables
  bind->bindNew("forceLocalize", force_localize, RhIO::Bind::PullOnly)
      ->comment("Special scanning mode: not looking nearby")
      ->defaultValue(false);
  bind->bindNew("forceScanBall", force_scan_ball, RhIO::Bind::PullOnly)
      ->comment("Force ball scanning mode")
      ->defaultValue(false);
  bind->bindNew("disabled", disabled, RhIO::Bind::PullOnly)
      ->comment("Target a fixed point, but still uses security")
      ->defaultValue(false);
  // Default scan parameters
  bind->bindNew("minTilt", min_tilt, RhIO::Bind::PullOnly)
      ->comment("Minimum tilt wished for an image point")
      ->defaultValue(-5);
  bind->bindNew("maxTilt", max_tilt, RhIO::Bind::PullOnly)
      ->comment("Maximum tilt wished for an image point")
      ->defaultValue(90);
  bind->bindNew("maxPan", max_pan, RhIO::Bind::PullOnly)
      ->comment("Maximum pan wished for an image point")
      ->defaultValue(110);
  bind->bindNew("minOverlap", min_overlap, RhIO::Bind::PullOnly)
      ->comment("Minimal overlap between control points [degrees]")
      ->defaultValue(15);
  // Localize scan parameters
  bind->bindNew("localizeMinTilt", localize_min_tilt, RhIO::Bind::PullOnly)
      ->comment("Minimum tilt wished for an image point")
      ->defaultValue(-8);
  bind->bindNew("localizeMaxTilt", localize_max_tilt, RhIO::Bind::PullOnly)
      ->comment("Maximum tilt wished for an image point")
      ->defaultValue(60);
  bind->bindNew("localizeMaxPan", localize_max_pan, RhIO::Bind::PullOnly)
      ->comment("Maximum pan wished for an image point")
      ->defaultValue(160);
  bind->bindNew("localizeMinOverlap", localize_min_overlap, RhIO::Bind::PullOnly)
      ->comment("Minimal overlap between control points [degrees]")
      ->defaultValue(5);
  // Speed and acc limits for orders
  // WARNING: persisted, because 'Tom' has a different camera requiring a higher shutter
  bind->bindNew("maxSpeed", max_speed, RhIO::Bind::PullOnly)
      ->comment("Maximal angular speed [deg/s]")
      ->defaultValue(210);
  bind->bindNew("maxAcc", max_acc, RhIO::Bind::PullOnly)->comment("Maximal acceleration [deg/s^2]")->defaultValue(3600);
  // Tracking
  bind->bindNew("maxTiltTrack", max_tilt_track, RhIO::Bind::PullOnly)
      ->comment("Maximum tilt wished for the center of the image when tracking")
      ->defaultValue(65);
  bind->bindNew("maxPanTrack", max_pan_track, RhIO::Bind::PullOnly)
      ->comment("Maximum pan wished for an image point when tracking")
      ->defaultValue(115);

  // Timing/tracking variables
  bind->bindNew("isTracking", is_tracking, RhIO::Bind::PushOnly)->comment("Is the robot currently tracking the ball");
  bind->bindNew("scanPeriod", scan_period, RhIO::Bind::PushOnly)->comment("Duration of scan cycle [s]");
  bind->bindNew("localizeScanPeriod", localize_scan_period, RhIO::Bind::PushOnly)
      ->comment("Duration of scan cycle in localize mode [s]");
  bind->bindNew("scanExtraPeriod", scan_extra_period, RhIO::Bind::PullOnly)
      ->comment("Extra time to ensure we are looking everywhere [s]")
      ->defaultValue(0.8);
  bind->bindNew("scanningTime", scanning_time, RhIO::Bind::PushOnly)
      ->comment("Time spent since start of scan phase [s]");
  bind->bindNew("trackingPeriod", tracking_period, RhIO::Bind::PullOnly)
      ->comment("Duration of tracking when ball_dist >= nearTrackDist [s]")
      ->defaultValue(1);
  bind->bindNew("nearTrackingPeriod", near_tracking_period, RhIO::Bind::PullOnly)
      ->comment("Duration of tracking when ball_dist < nearTrackDist [s]")
      ->defaultValue(3);
  bind->bindNew("trackingTime", tracking_time, RhIO::Bind::PushOnly)->comment("Time spent since start of tracking [s]");
  bind->bindNew("nearTrackDist", near_track_dist, RhIO::Bind::PullOnly)
      ->comment("Distance at which tracking becomes longer")
      ->defaultValue(1.0);
  bind->bindNew("forceTrackDist", force_track_dist, RhIO::Bind::PullOnly)
      ->comment("Distance to force watching the ball")
      ->defaultValue(0.6);
  bind->bindNew("forceTrack", force_track, RhIO::Bind::PullOnly)->comment("Is tracking forced ?")->defaultValue(false);

  // Targets monitored
  bind->bindNew("pan", pan_deg, RhIO::Bind::PushOnly)->comment("Target sent to head_yaw [deg]");
  bind->bindNew("tilt", tilt_deg, RhIO::Bind::PushOnly)->comment("Target sent to head_pitch [deg]");
  bind->bindNew("targetX", target_x, RhIO::Bind::PushOnly)->comment("Target position for center of camera [m]");
  bind->bindNew("targetY", target_y, RhIO::Bind::PushOnly)->comment("Target position for center of camera [m]");
  bind->bindNew("wishedPan", wished_pan, RhIO::Bind::PushOnly)->comment("Target provided by the scanner [deg]");
  bind->bindNew("wishedTilt", wished_tilt, RhIO::Bind::PushOnly)->comment("Target provided by the scanner [deg]");
  bind->bindNew("wishedDist", wished_dist, RhIO::Bind::PushOnly)
      ->comment("Distance of the point provided by the scanner [m]");

  bind->bindNew("predictedBallTimeOffset", predicted_ball_time_offset, RhIO::Bind::PullOnly)
      ->comment("Predicted ball in future [ms]")
      ->defaultValue(200.0);

  bind->bindNew("smoothing", smoothing, RhIO::Bind::PullOnly)
      ->comment("smoothing of the orders applied for both scan and track")
      ->defaultValue(0.9);

  bind->bindNew("fallBackwardPitch", fall_backward_pitch, RhIO::Bind::PullOnly)
      ->defaultValue(80)
      ->minimum(0)
      ->maximum(90)
      ->comment("Head pitch goal value during backward falls [deg]");
  bind->bindNew("fallForwardPitch", fall_forward_pitch, RhIO::Bind::PullOnly)
      ->defaultValue(-45)
      ->minimum(-50)
      ->maximum(0)
      ->comment("Head pitch goal value during forward falls [deg]");

  bind->pull();
}

void Head::setLocalizeMaxPan(double maxPan)
{
  bind->node().setFloat("localizeMaxPan", maxPan);
}

double Head::getScanPeriod() const
{
  return scan_period;
}

std::string Head::getName()
{
  return "head";
}

void Head::onStart()
{
  is_tracking = false;
  tracking_time = 0;
  scanning_time = 0;
}

void Head::step(float elapsed)
{
  // Pull variables from RhIO
  bind->pull();
  // Update Internal representation of the scanners
  updateScanners();

  updateTimers(elapsed);

  DecisionService* decision = getServices()->decision;

  // Use Model and camera parameters to determine position
  rhoban::HumanoidModel* model = &getServices()->model->model;
  const rhoban::CameraModel& camera_model = getServices()->model->cameraModel;

  Eigen::Vector3d target_in_self;
  if (disabled)
  {
    target_in_self = Eigen::Vector3d(1, 0, 0);
  }
  else if (force_localize)
  {
    target_in_self = getScanTarget(model, localize_scanner);
    is_tracking = false;  // Important to ensure scanning_time is properly updated
  }
  else if (shouldTrackBall())
  {
    target_in_self = getBallTarget(model, camera_model);
    is_tracking = true;
  }
  else
  {
    // If ball is properly localized, use the localization scanner rather than
    // the default scanner
    if (decision->isBallQualityGood && !force_scan_ball)
    {
      target_in_self = getScanTarget(model, localize_scanner);
    }
    else
    {
      target_in_self = getScanTarget(model, scanner);
    }
    is_tracking = false;
  }

  // Monitoring variables
  target_x = target_in_self[0];
  target_y = target_in_self[1];

  Eigen::Vector3d target_in_world = model->selfToWorld() * target_in_self;

  double wished_pan_rad, wished_tilt_rad;

  bool modelSuccess = model->cameraLookAt(wished_pan_rad, wished_tilt_rad, target_in_world);

  wished_pan = rad2deg(wished_pan_rad);
  wished_tilt = rad2deg(wished_tilt_rad);

  if (!is_tracking)
  {
    double max_diff = max_speed * elapsed;
    wished_pan = std::min(std::max(wished_pan, last_pan_target - max_diff), last_pan_target + max_diff);
    wished_tilt = std::min(std::max(wished_tilt, last_tilt_target - max_diff), last_tilt_target + max_diff);
  }

  last_pan_target = wished_pan;
  last_tilt_target = wished_tilt;

  if (!modelSuccess)
  {
    std::cout << "head: failed lookAtNoUpdate" << std::endl;
  }

  FallStatus fall_status = getServices()->decision->fallStatus;

  // if robot is fallen apply a specific procedure
  if (fall_status != FallStatus::Ok)
  {
    applyProtection();
  }
  else
  {
    // Smoothing orders
    pan_deg = smoothing * pan_deg + (1 - smoothing) * wished_pan;
    tilt_deg = smoothing * tilt_deg + (1 - smoothing) * wished_tilt;
    if (modelSuccess)
    {  // Never update pan_deg and tilt_deg if model failed
      // Bounding pan/tilt when tracking
      if (is_tracking)
      {
        pan_deg = std::max(-max_pan_track, std::min(max_pan_track, pan_deg));
      }
      // Absolute bounds on tilt for safety
      double min_tilt(-40), max_tilt(80);
      tilt_deg = std::min(max_tilt, std::max(min_tilt, tilt_deg));
      // Applying targets
      Move::setAngle("head_yaw", pan_deg);
      Move::setAngle("head_pitch", tilt_deg);
    }
    setTorqueLimit("head_pitch", 1.0);
  }

  bind->push();
}

void Head::setForceTrack(bool value)
{
  bind->node().setBool("forceTrack", value);
}

void Head::setForceScanBall(bool value)
{
  bind->node().setBool("forceScanBall", value);
}

void Head::setForceLocalize(bool value)
{
  bind->node().setBool("forceLocalize", value);
}
void Head::setDisabled(bool value)
{
  bind->node().setBool("disabled", value);
}
bool Head::isDisabled()
{
  return disabled;
}

void Head::updateTimers(float elapsed)
{
  if (is_tracking)
  {
    tracking_time += elapsed;
    scanning_time = 0;
  }
  else
  {
    scanning_time += elapsed;
    tracking_time = 0;
  }
}

bool Head::shouldTrackBall()
{
  // If tracking_period is 0, never restart
  if (tracking_period <= 0 || force_scan_ball)
    return false;
  // Otherwise: tracking depends on loc and history
  LocalisationService* loc = getServices()->localisation;
  DecisionService* decision = getServices()->decision;
  double ball_dist = loc->getBallPosSelf().getLength();

  // Never track ball if quality is too low
  if (!decision->isBallQualityGood)
    return false;

  // If the ball is close, we should track it, except if we are in freeze phase where we need to get some information
  // because the game can be blocked in this state for long
  RefereeService* referee = getServices()->referee;
  bool shouldTrackBecauseItsClose = ball_dist < force_track_dist && !referee->isFreezePhase();

  // For some cases, tracking is forced to stay active
  if (force_track || shouldTrackBecauseItsClose || decision->isBallMoving || decision->hasMateKickedRecently)
  {
    return true;
  }

  // If robot is tracking currently, stop tracking if period has ended
  if (is_tracking)
  {
    if (ball_dist < near_track_dist)
    {
      return tracking_time < near_tracking_period;
    }
    else
    {
      return tracking_time < tracking_period;
    }
  }
  // Otherwise, start tracking if scan_period is finished
  return scanning_time > scan_period + scan_extra_period;
}

Eigen::Vector3d Head::getScanTarget(rhoban::HumanoidModel* model, const HeadScan& scannerUsed)
{
  double robotHeight = model->position("camera", "support_foot").z();

  /// Reading wished pan/tilt for scanning
  Eigen::Vector2d wished_pan_tilt = scannerUsed.getTarget(scanning_time);
  wished_pan = wished_pan_tilt(0);
  wished_tilt = wished_pan_tilt(1);
  double wished_tilt_rad = deg2rad(wished_tilt);
  // Ensuring unlikely situations do not lead to SIGFPE
  double epsilon = 0.001;
  if (std::fabs(M_PI / 2 - wished_tilt_rad) < epsilon)
  {
    wished_dist = 0;
  }
  else
  {
    wished_dist = robotHeight / tan(wished_tilt_rad);
  }

  // If we ask for a negative tilt, revert to a positive dist, but target an artificial roof
  double targetZ = 0;
  if (wished_dist < 0)
  {
    wished_dist = -wished_dist;
    targetZ = 2 * robotHeight;
  }

  // Computing target in self referential
  Eigen::Vector3d target_in_self(wished_dist * cos(deg2rad(wished_pan)), wished_dist * sin(deg2rad(wished_pan)),
                                 targetZ);
  return target_in_self;
}

Eigen::Vector3d Head::getBallTarget(rhoban::HumanoidModel* model, const rhoban::CameraModel& camera_model)
{
  double robotHeight = model->position("camera", "support_foot").z();  //[m]

  LocalisationService* loc = getServices()->localisation;
  double futureTimestampMS = rhoban_utils::TimeStamp::now().getTimeMS() + predicted_ball_time_offset;
  auto point = loc->getPredictedBallSelf(rhoban_utils::TimeStamp::fromMS(futureTimestampMS));

  // Getting limits angle
  double max_tilt_value = max_tilt_track;
  double min_tilt_value = min_tilt + (camera_model.getFOVY() / 2).getSignedValue();

  // Bounding target
  double targetDist = std::sqrt(point.x * point.x + point.y * point.y);
  double targetTheta = std::atan2(point.y, point.x);
  double minDist = tan(M_PI / 2 - max_tilt_value * M_PI / 180) * robotHeight;
  double maxDist = tan(M_PI / 2 - min_tilt_value * M_PI / 180) * robotHeight;
  targetDist = std::min(maxDist, std::max(minDist, targetDist));
  point.x = cos(targetTheta) * targetDist;
  point.y = sin(targetTheta) * targetDist;

  return Eigen::Vector3d(point.x, point.y, 0);
}

void Head::updateScanners()
{
  const rhoban::CameraModel& camera_model = getServices()->model->cameraModel;
  double fovx = camera_model.getFOVX().getSignedValue();
  double fovy = camera_model.getFOVY().getSignedValue();
  // Update default scanner
  scanner.setFOV(fovx, fovy);
  scanner.setMinTilt(min_tilt);
  scanner.setMaxTilt(max_tilt);
  scanner.setMaxPan(max_pan);
  scanner.setMinOverlap(min_overlap);
  scanner.setControlLaw(max_speed, max_acc);
  // Update localisation scanner
  localize_scanner.setFOV(fovx, fovy);
  localize_scanner.setMinTilt(localize_min_tilt);
  localize_scanner.setMaxTilt(localize_max_tilt);
  localize_scanner.setMaxPan(localize_max_pan);
  localize_scanner.setMinOverlap(localize_min_overlap);
  localize_scanner.setControlLaw(max_speed, max_acc);

  // Update trajectory if parameters have changed
  scanner.synchronize();
  localize_scanner.synchronize();
  // Update scan periods parameters
  scan_period = scanner.getCycleDuration();
  localize_scan_period = localize_scanner.getCycleDuration();
}

void Head::applyProtection()
{
  DecisionService* decision = getServices()->decision;
  FallStatus fall_status = decision->fallStatus;
  FallDirection fall_direction = decision->fallDirection;
  if (fall_status == FallStatus::Falling || fall_status == FallStatus::Fallen)
  {
    tilt_deg = 0;
    switch (fall_direction)
    {
      case FallDirection::Forward:
        tilt_deg = fall_forward_pitch;
        break;
      case FallDirection::Backward:
        tilt_deg = fall_backward_pitch;
        break;
      default:
        break;
    }
    Move::setAngle("head_pitch", tilt_deg);
  }
  // if ( fall_status == FallStatus::Fallen) {
  //  Move::setAngle("head_pitch", tilt_deg);
  //  //setTorqueLimit("head_pitch", 0.0);
  //  //setTorqueLimit("head_yaw", 0.0);
  //}
  Move::setAngle("head_yaw", 0);
}
