#pragma once

#include "CameraState/CameraState.hpp"
#include "Localisation/RadarFilterPoint.hpp"

namespace Vision
{
namespace Localisation
{

class RobotFilter : public RadarFilterPoint
{
public:
  RobotFilter(Utils::CameraState *cs);

  bool bindToRhIO(std::string node, std::string command);

  bool defaultNormalizeSum() override;

  Eigen::Vector2d getGroundPosSelf(const Eigen::Vector3d & pt);

	bool isSimilar(const Eigen::Vector3d &pos1, const Eigen::Vector3d &pos2) override;

  Eigen::Vector3d mergeObjects(const Eigen::Vector3d & o1, const Eigen::Vector3d & o2,
                               double w1, double w2) override;

private:

  /// Tolerance to a pitch error for merging objects [deg]
  double pitch_tol;

  /// Tolerance to a direction error for merging objects [deg]
  double dir_tol;

  /// When merging, closest point has it's weight increased by this factor
  double closest_gain;
};

}
}
