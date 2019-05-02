#pragma once

#include <map>
#include <Eigen/Dense>
#include <LegIK/LegIK.hpp>
#include "Spline/CubicSpline.hpp"
#include "Model/HumanoidFixedModel.hpp"

namespace rhoban
{
class WalkEngine
{
public:
  // Foot position and speed in 2.5D
  struct FootPose
  {
    FootPose();

    double x, y, z;
    double yaw;
  };

  struct Foot
  {
    // Get the foot position, t is from 0 to 1, playing the footstep
    struct FootPose getPosition(double t);

    // Update splines for the foot step
    void clearSplines();
    
    // Splines
    Leph::CubicSpline xSpline, ySpline, zSpline, yawSpline;

    double trunkYOffset;
  };

  // Ticks the walk engine
  WalkEngine();
  void initByModel(Leph::HumanoidFixedModel& model);

  // Re-compute splines
  void reset();
  void updateSplines();
  double update(double timeSinceLastStep);
  void newStep();

  // Updating feet position
  void assignModel(Leph::HumanoidFixedModel& model);

  // Walk engine left and right feet position
  struct Foot left, right;

  // Support and flying foot
  bool isLeftSupport;
  struct Foot& supportFoot();
  struct Foot& flyingFoot();

  // Walk engine parameters
  double trunkXOffset;
  double trunkZOffset;
  double footYOffset;
  double riseGain;
  double riseDuration;
  double frequency;
  double swingGain;
  double swingPhase;
  double trunkHeight;
  double footDistance;
  double footYOffsetPerYSpeed;

  // Dynamics orders
  double xSpeed, ySpeed, yawSpeed;

  double _t;
  double halfPeriod;

  // Returns the phase of current step (0: begin to 1: end)
  double getStepPhase();

  // This is stored for the current step to avoid having it changing during the step itself
  double _swingGain;
};
}  // namespace rhoban