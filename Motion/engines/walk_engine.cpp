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
  xSpline = rhoban_utils::PolySpline();
  ySpline = rhoban_utils::PolySpline();
  zSpline = rhoban_utils::PolySpline();
  yawSpline = rhoban_utils::PolySpline();
}

WalkEngine::FootPose WalkEngine::Foot::getPosition(double t)
{
  FootPose pose;
  pose.x = xSpline.get(t);
  pose.y = ySpline.get(t);
  pose.z = zSpline.get(t);
  pose.yaw = yawSpline.get(t);

  return pose;
}

void WalkEngine::FootPose::operator=(const FootPose& other)
{
  x = other.x;
  y = other.y;
  z = other.z;
  yaw = other.yaw;
}

WalkEngine::FootPose::FootPose() : x(0), y(0), z(0), yaw(0)
{
}

WalkEngine::WalkEngine()
  : trunkXOffset(0.0)
  , trunkZOffset(0.02)
  , footYOffset(0.03)
  , riseGain(0.035)
  , riseDuration(0.2)
  , frequency(1.5)
  , swingGain(0.02)
  , swingPhase(0.0)
  , footYOffsetPerStepSizeY(0.1)
  , trunkPitch(0)
  , stepSizeX(0)
  , stepSizeY(0)
  , stepSizeYaw(0)
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

void WalkEngine::assignModel(Leph::HumanoidFixedModel& model, double timeSinceLastStep)
{
  if (timeSinceLastStep < 0)
  {
    timeSinceLastStep = 0;
  }
  if (timeSinceLastStep > stepDuration)
  {
    timeSinceLastStep = stepDuration;
  }

  FootPose leftPose = left.getPosition(timeSinceLastStep);
  FootPose rightPose = right.getPosition(timeSinceLastStep);

  bool success = true;

  // XXX: This could be done with a spline with more possible adjustements
  // Adding swing with a sine
  double swingP = isLeftSupport ? M_PI : 0;
  swingP += M_PI * 2 * swingPhase;
  double swing = _swingGain * sin(M_PI * timeSinceLastStep / stepDuration + swingP);

  Eigen::Affine3d trunkPitchRot(Eigen::AngleAxisd(-trunkPitch, Eigen::Vector3d::UnitY()));

  // XXX: Yaw in the trunk frame appear to behave in the wrong orientation here
  success = success && model.get().legIkLeft(
                           "trunk",
                           trunkPitchRot *
                               Eigen::Vector3d(leftPose.x, leftPose.y + swing, trunkHeight + trunkZOffset + leftPose.z),
                           trunkPitchRot.linear().inverse() * Leph::AxisToMatrix(Eigen::Vector3d(0, 0, -leftPose.yaw)));
  success = success && model.get().legIkRight("trunk",
                                              trunkPitchRot * Eigen::Vector3d(rightPose.x, rightPose.y + swing,
                                                                              trunkHeight + trunkZOffset + rightPose.z),
                                              trunkPitchRot.linear().inverse() *
                                                  Leph::AxisToMatrix(Eigen::Vector3d(0, 0, -rightPose.yaw)));

  if (!success)
  {
    std::cerr << "WalkEngine bad orders!" << std::endl;
  }
}

void WalkEngine::newStep()
{
  _swingGain = swingGain;
  double previousStepDuration = stepDuration;
  stepDuration = 1.0 / (2 * frequency);

  Foot oldLeft = left;
  Foot oldRight = right;
  left.clearSplines();
  right.clearSplines();

  left.trunkYOffset = footDistance + footYOffset + footYOffsetPerStepSizeY * fabs(stepSizeY);
  right.trunkYOffset = -(footDistance + footYOffset + footYOffsetPerStepSizeY * fabs(stepSizeY));

  // Defining begining of splines according to previous feet position
  left.xSpline.addPoint(0, oldLeft.xSpline.get(previousStepDuration), oldLeft.xSpline.getVel(previousStepDuration));
  left.ySpline.addPoint(0, oldLeft.ySpline.get(previousStepDuration), oldLeft.ySpline.getVel(previousStepDuration));
  left.yawSpline.addPoint(0, oldLeft.yawSpline.get(previousStepDuration),
                          oldLeft.yawSpline.getVel(previousStepDuration));

  right.xSpline.addPoint(0, oldRight.xSpline.get(previousStepDuration), oldRight.xSpline.getVel(previousStepDuration));
  right.ySpline.addPoint(0, oldRight.ySpline.get(previousStepDuration), oldRight.ySpline.getVel(previousStepDuration));
  right.yawSpline.addPoint(0, oldRight.yawSpline.get(previousStepDuration),
                           oldRight.yawSpline.getVel(previousStepDuration));

  // Changing support foot
  isLeftSupport = !isLeftSupport;

  // Support foot is on the ground
  supportFoot().zSpline.addPoint(0, 0, 0);
  supportFoot().zSpline.addPoint(stepDuration, 0, 0);

  // Flying foot will rise
  flyingFoot().zSpline.addPoint(0, 0, 0);
  if (riseDuration > 0)
  {
    flyingFoot().zSpline.addPoint(stepDuration * (0.5 - riseDuration / 2.0), riseGain, 0);
    flyingFoot().zSpline.addPoint(stepDuration * (0.5 + riseDuration / 2.0), riseGain, 0);
  }
  else
  {
    flyingFoot().zSpline.addPoint(stepDuration * 0.5, riseGain, 0);
  }
  flyingFoot().zSpline.addPoint(stepDuration, 0, 0);

  if (fabs(stepSizeYaw) > 0.01)
  {
    // Speed vector
    Point speed(stepSizeX, stepSizeY);
    // Center of rotation
    Point center = speed.perpendicular() / stepSizeYaw;

    // For both feet, computing the new position in the
    Point sFoot(trunkXOffset, supportFoot().trunkYOffset);
    sFoot = (sFoot - center).rotation(rad2deg(-stepSizeYaw / 2.0)) + center;
    Point sFootSpeed = -speed.rotation(rad2deg(-stepSizeYaw / 2.0));
    supportFoot().xSpline.addPoint(stepDuration, sFoot.x, sFootSpeed.x / stepDuration);
    supportFoot().ySpline.addPoint(stepDuration, sFoot.y, sFootSpeed.y / stepDuration);

    Point fFoot(trunkXOffset, flyingFoot().trunkYOffset);
    fFoot = (fFoot - center).rotation(rad2deg(stepSizeYaw / 2.0)) + center;
    Point fFootSpeed = -speed.rotation(rad2deg(stepSizeYaw / 2.0));
    flyingFoot().xSpline.addPoint(stepDuration, fFoot.x, fFootSpeed.x / stepDuration);
    flyingFoot().ySpline.addPoint(stepDuration, fFoot.y, fFootSpeed.y / stepDuration);
  }
  else
  {
    supportFoot().xSpline.addPoint(stepDuration, trunkXOffset - stepSizeX / 2.0, -stepSizeX / stepDuration);
    supportFoot().ySpline.addPoint(stepDuration, supportFoot().trunkYOffset - stepSizeY / 2.0,
                                   -stepSizeY / stepDuration);

    flyingFoot().xSpline.addPoint(stepDuration, trunkXOffset + stepSizeX / 2.0, -stepSizeX / stepDuration);
    flyingFoot().ySpline.addPoint(stepDuration, flyingFoot().trunkYOffset + stepSizeY / 2.0, -stepSizeY / stepDuration);
  }

  // Yaw spline
  supportFoot().yawSpline.addPoint(stepDuration, -stepSizeYaw / 2, -stepSizeYaw / stepDuration);
  flyingFoot().yawSpline.addPoint(stepDuration, stepSizeYaw / 2, -stepSizeYaw / stepDuration);
}

void WalkEngine::reset()
{
  left.trunkYOffset = footDistance + footYOffset;
  right.trunkYOffset = -(footDistance + footYOffset);

  isLeftSupport = false;

  stepDuration = 1.0 / (2 * frequency);
  left.xSpline.addPoint(stepDuration, 0, 0);
  left.ySpline.addPoint(stepDuration, left.trunkYOffset, 0);
  left.yawSpline.addPoint(stepDuration, 0, 0);
  right.xSpline.addPoint(stepDuration, 0, 0);
  right.ySpline.addPoint(stepDuration, right.trunkYOffset, 0);
  right.yawSpline.addPoint(stepDuration, 0, 0);

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
