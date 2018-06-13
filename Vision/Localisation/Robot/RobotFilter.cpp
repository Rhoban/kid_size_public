#include "RobotFilter.hpp"

#include <rhoban_utils/angle.h>
#include <rhoban_utils/logging/logger.h>

using namespace rhoban_utils;

static Logger logger("RobotFilter");

namespace Vision
{
namespace Localisation
{
        
RobotFilter::RobotFilter(Utils::CameraState *cs)
    : RadarFilterPoint(cs)
{
  // Overriding default values of parent classes
  initialScore = 0.2;
  maximumScore = 1.0;
  scoreIncrease = 0.1;
  scoreReduction = 0.01;
  scoreReductionOut = 0.001;
  positionDiscount = 0.8;
  normalizeSum = true;
  matchDistance = 1.0;
  far = 8;
}

bool RobotFilter::bindToRhIO(std::string node, std::string command)
{
  if (RadarFilterPoint::bindToRhIO(node,command)) {
    bind->bindNew("pitchTol", pitch_tol, RhIO::Bind::PullOnly)
      ->comment("Tolerance for obstacles along pitch [deg]")
      ->defaultValue(25);
    bind->bindNew("dirTol", dir_tol, RhIO::Bind::PullOnly)
      ->comment("Tolerance for obstacles along yaw [deg]")
      ->defaultValue(5);
    bind->bindNew("closestGain", closest_gain, RhIO::Bind::PullOnly)
      ->comment("Weight factor for the closest object when merging")
      ->defaultValue(5);
    return true;
  }
  return false;
}

Eigen::Vector2d RobotFilter::getGroundPosSelf(const Eigen::Vector3d & pt)
{
  return cameraState->_model->frameInSelf("origin", pt).segment(0,2);
}

bool RobotFilter::isSimilar(const Eigen::Vector3d &p1, const Eigen::Vector3d &p2)
{
  Eigen::Vector2d pos1 = getGroundPosSelf(p1);
  Eigen::Vector2d pos2 = getGroundPosSelf(p2);

  double ground_dist1 = pos1.norm();
  double ground_dist2 = pos2.norm();
  double robot_height = cameraState->getHeight(); 

  Angle dir1 = Angle::fromXY(pos1(0), pos1(1));
  Angle dir2 = Angle::fromXY(pos2(0), pos2(1));
  Angle pitch1 = Angle::fromXY(ground_dist1, -robot_height);
  Angle pitch2 = Angle::fromXY(ground_dist2, -robot_height);

  double dir_diff = std::fabs((dir1 - dir2).getSignedValue());
  double pitch_diff = std::fabs((pitch1 - pitch2).getSignedValue());

  return dir_diff < dir_tol && pitch_diff < pitch_tol;
}

Eigen::Vector3d RobotFilter::mergeObjects(const Eigen::Vector3d & o1,
                                          const Eigen::Vector3d & o2,
                                          double w1, double w2)
{
  Eigen::Vector2d pos1 = getGroundPosSelf(o1);
  Eigen::Vector2d pos2 = getGroundPosSelf(o2);

  if (pos1.norm() < pos2.norm()) {
    w1 *= closest_gain;
  } else {
    w2 *= closest_gain;
  }
  return (w1 * o1 + w2 * o2) / (w1+w2);
}

}
}
