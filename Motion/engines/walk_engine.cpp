#include "walk_engine.h"
#include "Utils/AxisAngle.h"
#include "rhoban_geometry/point.h"
#include "rhoban_utils/angle.h"

using rhoban_geometry::Point;
using rhoban_utils::rad2deg;

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
  , footYOffset(0.035)
  , riseGain(0.035)
  , riseDuration(0.2)
  , frequency(1.5)
  , swingGain(0.02)
  , swingPhase(0.0)
  , footYOffsetPerYSpeed(0.1)
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

  // XXX: This could be done with a spline with more possible adjustements
  // Adding swing with a sine
  double swingP = isLeftSupport ? M_PI : 0;
  swingP += M_PI * 2 * swingPhase;
  double swing = _swingGain * sin(M_PI * getStepPhase() + swingP);

  // XXX: Yaw in the trunk frame appear to behave in the wrong orientation here
  success =
      success && model.get().legIkLeft(
                     "trunk", Eigen::Vector3d(leftPose.x, leftPose.y + swing, trunkHeight + trunkZOffset + leftPose.z),
                     Leph::AxisToMatrix(Eigen::Vector3d(0, 0, -leftPose.yaw)));
  success = success &&
            model.get().legIkRight(
                "trunk", Eigen::Vector3d(rightPose.x, rightPose.y + swing, trunkHeight + trunkZOffset + rightPose.z),
                Leph::AxisToMatrix(Eigen::Vector3d(0, 0, -rightPose.yaw)));

  if (!success)
  {
    std::cerr << "WalkEngine bad orders!" << std::endl;
  }
}

double WalkEngine::update(double timeSinceLastStep)
{
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
  _swingGain = swingGain;
  double previousHalfPeriod = halfPeriod;
  halfPeriod = 1.0 / (2 * frequency);

  Foot oldLeft = left;
  Foot oldRight = right;
  left.clearSplines();
  right.clearSplines();

  left.trunkYOffset = footDistance + footYOffset + footYOffsetPerYSpeed * fabs(ySpeed);
  right.trunkYOffset = -(footDistance + footYOffset + footYOffsetPerYSpeed * fabs(ySpeed));

  // Defining begining of splines according to previous feet position
  left.xSpline.addPoint(0, oldLeft.xSpline.pos(previousHalfPeriod), oldLeft.xSpline.vel(previousHalfPeriod));
  left.ySpline.addPoint(0, oldLeft.ySpline.pos(previousHalfPeriod), oldLeft.ySpline.vel(previousHalfPeriod));
  left.yawSpline.addPoint(0, oldLeft.yawSpline.pos(previousHalfPeriod), oldLeft.yawSpline.vel(previousHalfPeriod));

  right.xSpline.addPoint(0, oldRight.xSpline.pos(previousHalfPeriod), oldRight.xSpline.vel(previousHalfPeriod));
  right.ySpline.addPoint(0, oldRight.ySpline.pos(previousHalfPeriod), oldRight.ySpline.vel(previousHalfPeriod));
  right.yawSpline.addPoint(0, oldRight.yawSpline.pos(previousHalfPeriod), oldRight.yawSpline.vel(previousHalfPeriod));

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
  flyingFoot().zSpline.addPoint(halfPeriod, 0, 0);

  if (fabs(yawSpeed) > 0.01)
  {
    // Speed vector
    Point speed(xSpeed, ySpeed);
    // Center of rotation
    Point center = speed.perpendicular() / yawSpeed;

    // For both feet, computing the new position in the
    Point sFoot(trunkXOffset, supportFoot().trunkYOffset);
    sFoot = (sFoot - center).rotation(rad2deg(-yawSpeed / 2.0)) + center;
    Point sFootSpeed = speed.rotation(rad2deg(-yawSpeed / 2.0));
    supportFoot().xSpline.addPoint(halfPeriod, sFoot.x, sFootSpeed.x);
    supportFoot().ySpline.addPoint(halfPeriod, sFoot.y, sFootSpeed.y);

    Point fFoot(trunkXOffset, flyingFoot().trunkYOffset);
    fFoot = (fFoot - center).rotation(rad2deg(yawSpeed / 2.0)) + center;
    Point fFootSpeed = -speed.rotation(rad2deg(yawSpeed / 2.0));
    flyingFoot().xSpline.addPoint(halfPeriod, fFoot.x, fFootSpeed.x);
    flyingFoot().ySpline.addPoint(halfPeriod, fFoot.y, fFootSpeed.y);
  }
  else
  {
    supportFoot().xSpline.addPoint(halfPeriod, trunkXOffset - xSpeed / 2.0, -xSpeed);
    supportFoot().ySpline.addPoint(halfPeriod, supportFoot().trunkYOffset - ySpeed / 2.0, -ySpeed);

    flyingFoot().xSpline.addPoint(halfPeriod, trunkXOffset + xSpeed / 2.0, -xSpeed);
    flyingFoot().ySpline.addPoint(halfPeriod, flyingFoot().trunkYOffset + ySpeed / 2.0, -ySpeed);
  }

  // Yaw spline
  supportFoot().yawSpline.addPoint(halfPeriod, -yawSpeed / 2, -yawSpeed);
  flyingFoot().yawSpline.addPoint(halfPeriod, yawSpeed / 2, -yawSpeed);
}

void WalkEngine::reset()
{
  left.trunkYOffset = footDistance + footYOffset;
  right.trunkYOffset = -(footDistance + footYOffset);

  isLeftSupport = false;

  halfPeriod = 1.0 / (2 * frequency);
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

double WalkEngine::getStepPhase()
{
  return _t / halfPeriod;
}
}  // namespace rhoban