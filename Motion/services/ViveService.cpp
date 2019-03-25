#include "services/ViveService.h"

#include <vive_provider/utils.h>
#include <rhoban_utils/util.h>
#include <rhoban_utils/timing/time_stamp.h>

using namespace vive_provider;

ViveService::ViveService()
  : vive_manager(getDefaultPort(), -1)
  , tracker_id(-1)
{
}

bool ViveService::tick(double elapsed)
{
  if (isActive()) {
    vive_manager.autoUpdateOffset();
  }
  return true;
}

bool ViveService::isActive() const
{
  return tracker_id > 0 && vive_manager.getMessages().size() > 0;
}

Eigen::Affine3d ViveService::getFieldToVive(uint64_t time_stamp, bool system_clock)
{
  if (!system_clock)
  {
    time_stamp += rhoban_utils::getSteadyClockOffset();
  }
  GlobalMsg vive_status = vive_manager.getMessage(time_stamp, system_clock);

  for (const TrackerMsg & tracker : vive_status.trackers())
  {
    if (tracker.tracker_idx() == tracker_id)
    {
      //TODO: handle offsets in tracker
      return vive_provider::getWorldToTracker(tracker);
    }
  }
  throw std::out_of_range(DEBUG_INFO + " no entry with id: " + std::to_string(tracker_id));
}
