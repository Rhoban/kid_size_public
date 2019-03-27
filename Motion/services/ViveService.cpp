#include "services/ViveService.h"

#include <vive_provider/utils.h>
#include <rhoban_utils/util.h>
#include <rhoban_utils/timing/time_stamp.h>

using namespace vive_provider;

ViveService::ViveService()
  : vive_manager(getDefaultPort(), -1)
  , bind("vive")
{
  bind.bindNew("camera_x", camera_x, RhIO::Bind::PullOnly)
    ->comment("camera position along x-axis in vive_referential [m]")
    ->defaultValue(0.0);
  bind.bindNew("camera_y", camera_y, RhIO::Bind::PullOnly)
    ->comment("camera position along y-axis in vive_referential [m]")
    ->defaultValue(0.0);
  bind.bindNew("camera_z", camera_z, RhIO::Bind::PullOnly)
    ->comment("camera position along z-axis in vive_referential [m]")
    ->defaultValue(0.0);
  bind.bindNew("camera_roll", camera_roll, RhIO::Bind::PullOnly)
    ->comment("Rotation along x-axis [deg]")
    ->defaultValue(0);
  bind.bindNew("camera_pitch", camera_pitch, RhIO::Bind::PullOnly)
    ->comment("Rotation along y-axis [deg]")
    ->defaultValue(0);
  bind.bindNew("camera_yaw", camera_yaw, RhIO::Bind::PullOnly)
    ->comment("Rotation along z-axis [deg]")
    ->defaultValue(0);
  bind.bindNew("tracker_serial", tracker_serial, RhIO::Bind::PullOnly)
    ->comment("Tracker serial number")
    ->persisted(true)
    ->defaultValue("");
  bind.bindFunc("vive", "View vive status", &ViveService::cmdVive, *this);

  bind.pull();
}

bool ViveService::tick(double elapsed)
{
  bind.pull();
  if (isActive()) {
    vive_manager.autoUpdateOffset();
  }
  bind.push();
  return true;
}

bool ViveService::isActive() const
{
  return tracker_serial != "" && vive_manager.getMessages().size() > 0;
}

std::string ViveService::cmdVive()
{
  std::ostringstream oss;

  uint64_t ts = rhoban_utils::TimeStamp::now().getTimeMS()*1000.0;
  Eigen::Affine3d transform = getFieldToVive(ts);
  Eigen::Matrix4d m = transform.matrix();
  Eigen::Matrix3d m3 = m.block(0, 0, 3, 3);
  Eigen::Vector3d rpy = m3.eulerAngles(0, 1, 2);

  oss << "- X: " << m(0, 3) << std::endl;
  oss << "- Y: " << m(1, 3) << std::endl;
  oss << "- Z: " << m(2, 3) << std::endl;
  oss << "- Roll: " << (rpy[0]*180.0/M_PI) << std::endl;
  oss << "- Pitch: " << (rpy[1]*180.0/M_PI) << std::endl;
  oss << "- Yaw: " << (rpy[2]*180.0/M_PI) << std::endl;
  oss << m << std::endl;

  return oss.str();
}

Eigen::Affine3d ViveService::getFieldToVive(uint64_t time_stamp, bool system_clock) const
{
  if (!system_clock)
  {
    time_stamp += rhoban_utils::getSteadyClockOffset();
  }
  GlobalMsg vive_status = vive_manager.getMessage(time_stamp, system_clock);

  for (const TrackerMsg & tracker : vive_status.trackers())
  {
    if (tracker.serial_number() == tracker_serial)
    {
      return vive_provider::getWorldToTracker(tracker);
    }
  }

  return Eigen::Affine3d::Identity();
  // XXX: This should be catched by the camera state (temporary fix above)
  // throw std::out_of_range(DEBUG_INFO + " no entry with serial: " + tracker_serial);
}

Eigen::Affine3d ViveService::getFieldToCamera(uint64_t time_stamp, bool system_clock) const
{
  return getViveToCamera() * getFieldToVive(time_stamp, system_clock);
}

Eigen::Affine3d ViveService::getViveToCamera() const
{
  Eigen::Vector3d center(camera_x, camera_y, camera_z);
  Eigen::Matrix3d rotation;
  rotation =
    Eigen::AngleAxisd(camera_yaw * M_PI / 180, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(camera_pitch * M_PI / 180, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(camera_roll * M_PI / 180, Eigen::Vector3d::UnitX());
  return Eigen::Affine3d(rotation) * Eigen::Translation3d(-center);
}

void ViveService::setPosOffset(const Eigen::Vector3d & pos)
{
  camera_x = pos.x();
  camera_y = pos.y();
  camera_z = pos.z();
}

void ViveService::setRoll(double roll)
{
  camera_roll = roll;
}
void ViveService::setPitch(double pitch)
{
  camera_pitch = pitch;
}

void ViveService::setYaw(double yaw)
{
  camera_yaw = yaw;
}
