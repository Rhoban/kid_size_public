#include "BallStackFilter.hpp"

#include "CameraState/CameraState.hpp"

#include <scheduler/MoveScheduler.h>

#include "rhoban_utils/angle.h"

using namespace Vision::Utils;
using namespace rhoban_utils;

namespace Vision
{
namespace Localisation
{
BallStackFilter::BallStackFilter(Utils::CameraState* cameraState) : RadarFilterPoint(cameraState)
{
  lastKick = TimeStamp::now();
}

bool BallStackFilter::bindToRhIO(std::string node, std::string command)
{
  if (RadarFilterPoint::bindToRhIO(node, command))
  {
    bind->bindNew("afterKickPermissiveDuration", afterKickPermissiveDuration, RhIO::Bind::PullOnly)
        ->defaultValue(3.0)
        ->comment("Duration (s) of permissive merge after kick");

    return true;
  }

  return false;
}

void BallStackFilter::reset(float x, float y)
{
  mutex.lock();
  candidates.clear();

  Eigen::Vector3d point(x, y, 0.0);
  Candidate candidate;
  candidate.object = cameraState->getWorldFromSelf(point);
  candidate.score = maximumScore;
  candidates.push_back(candidate);

  mutex.unlock();
}

void BallStackFilter::applyKick(float x, float y)
{
  mutex.lock();

  lastKick = TimeStamp::now();

  std::vector<Candidate> new_candidates;

  for (Candidate candidate : candidates)
  {
    Candidate kicked_candidate = candidate;
    auto pos = cameraState->getSelfFromWorld(candidate.object);
    pos.x() += x;
    pos.y() += y;
    kicked_candidate.object = cameraState->getWorldFromSelf(pos);
    if (duplicateOnKick)
    {
      candidate.score /= 2;
      kicked_candidate.score /= 2;
      new_candidates.push_back(candidate);
      new_candidates.push_back(kicked_candidate);
    }
    else
    {
      new_candidates.push_back(kicked_candidate);
    }
  }

  candidates.clear();
  for (const Candidate& c : new_candidates)
  {
    candidates.push_back(c);
  }

  mutex.unlock();
}

bool BallStackFilter::isSimilar(const Eigen::Vector3d& pt1, const Eigen::Vector3d& pt2)
{
  auto now = TimeStamp::now();
  if (diffMs(lastKick, now) < afterKickPermissiveDuration * 1000)
  {
    return true;
  }

  return RadarFilterPoint::isSimilar(pt1, pt2);
}

}  // namespace Localisation
}  // namespace Vision
