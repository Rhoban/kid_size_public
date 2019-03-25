#include "services/ViveService.h"

#include <vive_provider/utils.h>
#include <rhoban_utils/util.h>
#include <rhoban_utils/timing/time_stamp.h>

using namespace vive_provider;

ViveService::ViveService()
  : vive_manager(getDefaultPort(), -1)
  , tracker_id(-1)
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
    ->defaultValue(90);
  bind.bindNew("camera_pitch", camera_pitch, RhIO::Bind::PullOnly)
    ->comment("Rotation along y-axis [deg]")
    ->defaultValue(90);
  bind.bindNew("camera_yaw", camera_yaw, RhIO::Bind::PullOnly)
    ->comment("Rotation along z-axis [deg]")
    ->defaultValue(0);
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
  return tracker_id > 0 && vive_manager.getMessages().size() > 0;
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
    if ((int)tracker.tracker_idx() == tracker_id)
    {
      return vive_provider::getWorldToTracker(tracker);
    }
  }
  throw std::out_of_range(DEBUG_INFO + " no entry with id: " + std::to_string(tracker_id));
}

Eigen::Affine3d ViveService::getFieldToCamera(uint64_t time_stamp, bool system_clock) const
{
  return getViveToCamera() * getFieldToVive(time_stamp, system_clock);
}

Eigen::Affine3d ViveService::getViveToCamera() const
{
  Eigen::Vector3d center(camera_x, camera_y, camera_z);
  Eigen::Matrix3d rotation;
  rotation = Eigen::AngleAxisd(camera_yaw * M_PI / 180, Eigen::Vector3d::UnitZ()) *
             Eigen::AngleAxisd(camera_pitch * M_PI / 180, Eigen::Vector3d::UnitY()) *
             Eigen::AngleAxisd(camera_roll * M_PI / 180, Eigen::Vector3d::UnitX());
  return Eigen::Translation3d(center) * Eigen::Affine3d(rotation);
}
