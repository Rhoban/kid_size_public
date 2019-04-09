#include <rhoban_geometry/point.h>
#include <Eigen/Dense>
#include "RadarFilterPoint.hpp"

#include "Utils/Interface.h"

using namespace Vision::Utils;
using namespace rhoban_utils;

namespace Vision
{
namespace Localisation
{
RadarFilterPoint::RadarFilterPoint(CameraState* cameraState)
  : RadarFilter(), cameraState(cameraState), matchDistance(0.4), matchAngle(2), alignedAngle(60), far(2)
{
}

bool RadarFilterPoint::bindToRhIO(std::string node, std::string command)
{
  if (RadarFilter<Eigen::Vector3d>::bindToRhIO(node, command))
  {
    bind->bindNew("matchDistance", matchDistance, RhIO::Bind::PullOnly)
        ->comment("Radius to match the same object")
        ->defaultValue(matchDistance);

    bind->bindNew("far", far, RhIO::Bind::PullOnly)
        ->comment("Distance where the object is considered far")
        ->defaultValue(far);

    bind->bindNew("matchAngle", matchAngle, RhIO::Bind::PullOnly)
        ->comment("Angle at which we match objects [deg]")
        ->defaultValue(matchAngle);

    bind->bindNew("alignedAngle", alignedAngle, RhIO::Bind::PullOnly)
        ->comment("Angle until which objects are considered as visible [deg]")
        ->defaultValue(alignedAngle);

    return true;
  }

  return false;
}

bool RadarFilterPoint::isVisible(const Eigen::Vector3d& pt)
{
  // Point is in robot basis in order to check if ball is seen
  cv::Point2f pt_cv(pt.x(), pt.y());
  try
  {
    cv::Point2f point = cameraState->imgXYFromWorldPosition(pt_cv);
    bool inScreen = cameraState->getCameraModel().containsPixel(point);

    auto pos = cameraState->getSelfFromWorld(pt);
    bool isFar = pos.norm() > far;
    // When objects are excentred and close, there is a risk to have them
    Angle objDir = Angle::fromXY(pos.x(), pos.y());
    bool isAligned = objDir.getSignedValue() < alignedAngle;

    return inScreen && !isFar && isAligned;
  }
  catch (const std::runtime_error& exc)
  {
    // Fails when the object is behind the camera which means point is not visible
    return false;
  }
}

bool RadarFilterPoint::isSimilar(const Eigen::Vector3d& pt1, const Eigen::Vector3d& pt2)
{
  // We need the position in robot referential, c1 and c2 are in world referential
  Eigen::Vector3d c1_robot = cameraState->getSelfFromWorld(pt1);
  Eigen::Vector3d c2_robot = cameraState->getSelfFromWorld(pt2);
  double robotHeight = cameraState->getHeight();

  // Converting to Point3f
  cv::Point3f candidateDir, seenDir;
  candidateDir.x = c1_robot(0);
  candidateDir.y = c1_robot(1);
  candidateDir.z = -robotHeight;
  seenDir.x = c2_robot(0);
  seenDir.y = c2_robot(1);
  seenDir.z = -robotHeight;

  // Computing angle and pos differences
  Angle aDiff = angleBetween(seenDir, candidateDir);
  double posDiff = (c1_robot - c2_robot).norm();

  return posDiff < matchDistance || fabs(aDiff.getSignedValue()) < matchAngle;
}

std::string RadarFilterPoint::toString(const Eigen::Vector3d& point)
{
  std::stringstream ss;

  auto pos = cameraState->getSelfFromWorld(point);

  ss << pos.x() << ", " << pos.y();

  return ss.str();
}

void RadarFilterPoint::updateCS(Vision::Utils::CameraState* newCS)
{
  cameraState = newCS;
}

}  // namespace Localisation
}  // namespace Vision
