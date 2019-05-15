#pragma once

#include <rhoban_geometry/point.h>
#include <Eigen/Dense>
#include <CameraState/CameraState.hpp>
#include "RadarFilter.hpp"

namespace Vision
{
namespace Localisation
{
class RadarFilterPoint : public RadarFilter<Eigen::Vector3d>
{
public:
  RadarFilterPoint(Vision::Utils::CameraState* cameraState);

  virtual bool bindToRhIO(std::string node, std::string command);

  virtual bool isVisible(const Eigen::Vector3d& point);

  virtual bool isSimilar(const Eigen::Vector3d& pt1, const Eigen::Vector3d& pt2);

  virtual std::string toString(const Eigen::Vector3d& point);

  virtual void updateCS(Vision::Utils::CameraState* newCS);

protected:
  // Dependencies
  Vision::Utils::CameraState* cameraState;

  // Merge thresholds
  float matchDistance;
  float matchAngle;
  float alignedAngle;

  // Is the object far ?
  float far;
};

}  // namespace Localisation
}  // namespace Vision
