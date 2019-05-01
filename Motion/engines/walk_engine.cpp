#include "walk_engine.h"
#include "Utils/AxisAngle.h"
#include "rhoban_geometry/point.h"
#include "rhoban_utils/angle.h"

using rhoban_utils::rad2deg;
using rhoban_geometry::Point;

namespace rhoban
{
void WalkEngine::Foot::clearSplines()
{
  xSpline = Leph::CubicSpline();
  ySpline = Leph::CubicSpline();
  zSpline = Leph::CubicSpline();
  yawSpline = Leph::CubicSpline();
}

WalkEngine::FootPose WalkEngine::Foot::getPosition(double t)
{
  FootPose pose;
  pose.x = xSpline.pos(t);
  pose.y = ySpline.pos(t);
  pose.z = zSpline.pos(t);
  pose.yaw = yawSpline.pos(t);

  return pose;
}

WalkEngine::FootPose::FootPose() : x(0), y(0), z(0), yaw(0)
{
}

WalkEngine::WalkEngine()
  : trunkXOffset(0.0)
  , trunkZOffset(0.02)
  , footYOffset(0.025)
  , riseGain(0.04)
  , riseDuration(0)
  , frequency(1.5)
  , swingGain(0.5)
  , xSpeed(0)
  , ySpeed(0)
  , yawSpeed(0)
  , _t(0)
{
}

void WalkEngine::initByModel(Leph::HumanoidFixedModel& model)
{
  // Getting the trunk height and foot distance in zero position from URDF model
  model.get().setDOFZeros();
  trunkHeight = model.get().position("left_foot_tip", "trunk").z();
  footDistance = model.get().position("left_foot_tip", "trunk").y();

  reset();
}

void WalkEngine::assignModel(Leph::HumanoidFixedModel& model)
{
  FootPose leftPose = left.getPosition(_t);
  FootPose rightPose = right.getPosition(_t);

  bool success = true;
  success = success && model.get().legIkLeft(
                           "trunk", Eigen::Vector3d(leftPose.x, leftPose.y, trunkHeight + trunkZOffset + leftPose.z),
                           Leph::AxisToMatrix(Eigen::Vector3d(0, 0, leftPose.yaw)));
  success = success && model.get().legIkRight(
                           "trunk", Eigen::Vector3d(rightPose.x, rightPose.y, trunkHeight + trunkZOffset + rightPose.z),
                           Leph::AxisToMatrix(Eigen::Vector3d(0, 0, rightPose.yaw)));
  if (!success)
  {
    std::cerr << "WalkEngine bad orders!" << std::endl;
  }
}

double WalkEngine::update(double timeSinceLastStep)
{
  double halfPeriod = 1.0 / (2 * frequency);
  _t = 0;

  // Normalizing t between 0 and 1
  if (timeSinceLastStep > 0)
  {
    _t = timeSinceLastStep;
  }
  if (_t > halfPeriod)
  {
    _t = halfPeriod;
  }

  return timeSinceLastStep - halfPeriod;
}

void WalkEngine::newStep()
{
  double halfPeriod = 1.0 / (2 * frequency);
  Foot oldLeft = left;
  Foot oldRight = right;
  left.clearSplines();
  right.clearSplines();
  left.halfPeriod = halfPeriod;
  right.halfPeriod = halfPeriod;

  left.trunkYOffset = footDistance + footYOffset;
  right.trunkYOffset = -(footDistance + footYOffset);

  // Defining begining of splines according to previous feet position
  left.xSpline.addPoint(0, oldLeft.xSpline.pos(oldLeft.halfPeriod), oldLeft.xSpline.vel(oldLeft.halfPeriod));
  left.ySpline.addPoint(0, oldLeft.ySpline.pos(oldLeft.halfPeriod), oldLeft.ySpline.vel(oldLeft.halfPeriod));
  left.yawSpline.addPoint(0, oldLeft.yawSpline.pos(oldLeft.halfPeriod), oldLeft.yawSpline.vel(oldLeft.halfPeriod));

  right.xSpline.addPoint(0, oldRight.xSpline.pos(oldRight.halfPeriod), oldRight.xSpline.vel(oldRight.halfPeriod));
  right.ySpline.addPoint(0, oldRight.ySpline.pos(oldRight.halfPeriod), oldRight.ySpline.vel(oldRight.halfPeriod));
  right.yawSpline.addPoint(0, oldRight.yawSpline.pos(oldRight.halfPeriod), oldRight.yawSpline.vel(oldRight.halfPeriod));
  
  // Changing support foot
  isLeftSupport = !isLeftSupport;

  // Support foot is on the ground
  supportFoot().zSpline.addPoint(0, 0, 0);
  supportFoot().zSpline.addPoint(halfPeriod, 0, 0);

  // Flying foot will rise
  flyingFoot().zSpline.addPoint(0, 0, 0);
  if (riseDuration > 0)
  {
    flyingFoot().zSpline.addPoint(halfPeriod * (0.5 - riseDuration / 2.0), riseGain, 0);
    flyingFoot().zSpline.addPoint(halfPeriod * (0.5 + riseDuration / 2.0), riseGain, 0);
  }
  else
  {
    flyingFoot().zSpline.addPoint(halfPeriod * 0.5, riseGain, 0);
  }
  // flyingFoot().zSpline.addPoint(halfPeriod*0.5, riseGain, 0);
  // flyingFoot().zSpline.addPoint(halfPeriod*(0.5-xxx), riseGain, 0);
  // flyingFoot().zSpline.addPoint(halfPeriod*(0.5+xxx), riseGain, 0);
  flyingFoot().zSpline.addPoint(halfPeriod, 0, 0);

  Eigen::Vector2d supportTrunkXOffset(cos(-yawSpeed/2)*trunkXOffset, sin(-yawSpeed/2)*trunkXOffset);
  Eigen::Vector2d flyingTrunkXOffset(cos(yawSpeed/2)*trunkXOffset, sin(yawSpeed/2)*trunkXOffset);

  if (fabs(yawSpeed) > 0.01)
  {
    // Speed vector
    Point speed(xSpeed, ySpeed);
    // Center of rotation
    Point center = speed.perpendicular() / yawSpeed;

    // For both feet, computing the new position in the 
    Point sFoot(trunkXOffset, supportFoot().trunkYOffset);
    sFoot = (sFoot + center).rotation(rad2deg(yawSpeed / 2.0)) - center;
    Point sFootSpeed = speed.rotation(rad2deg(yawSpeed / 2.0));
    supportFoot().xSpline.addPoint(halfPeriod, sFoot.x, sFootSpeed.x);
    supportFoot().ySpline.addPoint(halfPeriod, sFoot.y, sFootSpeed.y);

    Point fFoot(trunkXOffset, flyingFoot().trunkYOffset);
    fFoot = (fFoot + center).rotation(rad2deg(-yawSpeed / 2.0)) - center;
    Point fFootSpeed = -speed.rotation(rad2deg(-yawSpeed / 2.0));
    flyingFoot().xSpline.addPoint(halfPeriod, fFoot.x, fFootSpeed.x);
    flyingFoot().ySpline.addPoint(halfPeriod, fFoot.y, fFootSpeed.y);
  }
  else
  {
    supportFoot().xSpline.addPoint(halfPeriod, supportTrunkXOffset.x()-xSpeed / 2.0, -xSpeed);
    supportFoot().ySpline.addPoint(halfPeriod, supportTrunkXOffset.y()+supportFoot().trunkYOffset - ySpeed / 2.0, -ySpeed);

    flyingFoot().xSpline.addPoint(halfPeriod, flyingTrunkXOffset.x() + xSpeed / 2.0, -xSpeed);
    flyingFoot().ySpline.addPoint(halfPeriod, flyingTrunkXOffset.y() + flyingFoot().trunkYOffset + ySpeed / 2.0, -ySpeed);
  }

  // Yaw spline
  supportFoot().yawSpline.addPoint(halfPeriod, -yawSpeed / 2, -yawSpeed);
  supportFoot().yawSpline.addPoint(halfPeriod, yawSpeed / 2, yawSpeed);
}

void WalkEngine::reset()
{
  left.trunkYOffset = footDistance + footYOffset;
  right.trunkYOffset = -(footDistance + footYOffset);

  isLeftSupport = false;

  double halfPeriod = 1.0 / (2 * frequency);
  left.halfPeriod = halfPeriod;
  right.halfPeriod = halfPeriod;
  left.xSpline.addPoint(halfPeriod, 0);
  left.ySpline.addPoint(halfPeriod, left.trunkYOffset);
  left.yawSpline.addPoint(halfPeriod, 0);
  right.xSpline.addPoint(halfPeriod, 0);
  right.ySpline.addPoint(halfPeriod, right.trunkYOffset);
  right.yawSpline.addPoint(halfPeriod, 0);

  newStep();
}

WalkEngine::Foot& WalkEngine::supportFoot()
{
  return isLeftSupport ? left : right;
}

WalkEngine::Foot& WalkEngine::flyingFoot()
{
  return isLeftSupport ? right : left;
}
}  // namespace rhoban