#include "walk_engine.h"

namespace rhoban
{
void WalkEngine::Foot::updateSplines()
{
  xSpline = Leph::CubicSpline();
  ySpline = Leph::CubicSpline();
  zSpline = Leph::CubicSpline();
  yawSpline = Leph::CubicSpline();

  double t = 0;
  for (StepState state : {StepBegin, StepMid, StepEnd}) {
    xSpline.addPoint(t, poses[state].x, poses[state].xVel);
    ySpline.addPoint(t, poses[state].y, poses[state].yVel);
    zSpline.addPoint(t, poses[state].z, poses[state].zVel);
    yawSpline.addPoint(t, poses[state].yaw, poses[state].yawVel);
    // std::cout << "Spline t: " << t << ", x: " << poses[state].x << std::endl;
    t += 0.5;
  }
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

WalkEngine::FootPose::FootPose() : x(0), y(0), z(0), yaw(0), xVel(0), yVel(0), zVel(0), yawVel(0)
{
}

WalkEngine::WalkEngine()
  : trunkZOffset(0.02), footYOffset(0.025), riseGain(0.04), frequency(3), xSpeed(0), ySpeed(0), yawSpeed(0), _t(0)
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
  success = success && model.get().legIkLeft("trunk", 
        Eigen::Vector3d(leftPose.x, leftPose.y, trunkHeight+trunkZOffset+leftPose.z));
  success = success && model.get().legIkRight("trunk",
         Eigen::Vector3d(rightPose.x, rightPose.y, trunkHeight+trunkZOffset+rightPose.z));
  if (!success) {
    std::cerr << "WalkEngine bad orders!" << std::endl;
  }
}

double WalkEngine::update(double timeSinceLastStep)
{
  double period = 1.0/frequency;
  _t = 0;

  // Normalizing t between 0 and 1
  if (timeSinceLastStep > 0) {
    _t = timeSinceLastStep / period;
  }
  if (_t > 1) {
    _t = 1;
  }

  return timeSinceLastStep - period;
}

void WalkEngine::newStep()
{
  left.trunkYOffset = footDistance + footYOffset;
  right.trunkYOffset = -(footDistance + footYOffset);

  // Pose of the feet at the begining of new step are the ones from the end of last step
  left.poses[StepBegin] = left.poses[StepEnd];
  right.poses[StepBegin] = right.poses[StepEnd];

  // Changing support foot
  isLeftSupport = !isLeftSupport;

  // Support foot is on the ground
  supportFoot().poses[StepMid].z = 0;
  supportFoot().poses[StepMid].zVel = 0;
  supportFoot().poses[StepEnd].z = 0;
  supportFoot().poses[StepEnd].zVel = 0;

  // Support foot at midpoint is at initial position
  supportFoot().poses[StepMid].x = 0;
  supportFoot().poses[StepMid].xVel = -xSpeed;
  supportFoot().poses[StepEnd].x = -xSpeed/2.0;
  supportFoot().poses[StepEnd].xVel = -xSpeed;

  // Flying foot will rise
  flyingFoot().poses[StepMid].z = riseGain;
  flyingFoot().poses[StepMid].zVel = 0;
  flyingFoot().poses[StepEnd].z = 0;
  flyingFoot().poses[StepEnd].zVel = 0;

  // FLying foot trajectory
  flyingFoot().poses[StepMid].x = 0;
  flyingFoot().poses[StepMid].xVel = xSpeed;
  flyingFoot().poses[StepEnd].x = xSpeed/2.0;
  flyingFoot().poses[StepEnd].xVel = -xSpeed;

  // std::cout << "* Update left spline" << std::endl;
  left.updateSplines();
  // std::cout << "* Update right spline" << std::endl;
  right.updateSplines();
}

void WalkEngine::reset()
{
  left.trunkYOffset = footDistance + footYOffset;
  right.trunkYOffset = -(footDistance + footYOffset);

  // Left is support foot
  isLeftSupport = false;

  left.poses.clear();
  right.poses.clear();

  FootPose leftInit, rightInit;
  leftInit.y = left.trunkYOffset;
  rightInit.y = right.trunkYOffset;

  left.poses[StepBegin] = leftInit;
  left.poses[StepMid] = leftInit;
  left.poses[StepEnd] = leftInit;
  right.poses[StepBegin] = rightInit;
  right.poses[StepMid] = rightInit;
  right.poses[StepEnd] = rightInit;

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