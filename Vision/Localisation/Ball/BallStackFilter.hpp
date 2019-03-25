#pragma once

#include <mutex>
#include <vector>
#include <deque>
#include <RhIO.hpp>
#include <Eigen/Dense>
#include <rhoban_utils/timing/time_stamp.h>
#include <Localisation/RadarFilterPoint.hpp>

namespace Vision
{
namespace Utils
{
class CameraState;
}

namespace Localisation
{
class BallStackFilter : public RadarFilterPoint
{
public:
  BallStackFilter(Utils::CameraState* cameraState);

  virtual bool bindToRhIO(std::string node, std::string command = "");

  // For permissive merge after kick
  virtual bool isSimilar(const Eigen::Vector3d& pt1, const Eigen::Vector3d& pt2);

  // Reseting the balls
  void reset(float x, float y);

  // Apply a kick
  void applyKick(float x, float y);

protected:
  ::rhoban_utils::TimeStamp lastKick;

  // Parameters
  float afterKickPermissiveDuration;
  bool duplicateOnKick;
};
}  // namespace Localisation
}  // namespace Vision
