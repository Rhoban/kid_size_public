#pragma once

#include <map>
#include <Eigen/Dense>
#include "rhoban_utils/spline/poly_spline.h"
#include "robot_model/humanoid_model.h"

namespace rhoban
{
class WalkEngine
{
public:
  // Foot position and speed in 2.5D
  struct FootPose
  {
    FootPose();
    void operator=(const FootPose& other);

    double x, y, z;
    double yaw;

    // Returns a frame from foot to trunk frame (this is actually a 2d matrix)
    Eigen::Affine3d footToTrunk();
  };

  struct Foot
  {
    // Get the foot position, t is from 0 to 1, playing the footstep
    struct FootPose getPosition(double t);

    // Update splines for the foot step
    void clearSplines();

    // Splines
    rhoban_utils::PolySpline xSpline, ySpline, zSpline, yawSpline;

    double trunkYOffset;
  };

  // Ticks the walk engine
  WalkEngine();
  void initByModel(rhoban::HumanoidModel& model);

  // Re-compute splines
  void reset();
  void updateSplines();
  void newStep();

  // Updating feet position
  std::map<std::string, double> computeAngles(rhoban::HumanoidModel& model, double timeSinceLastStep);

  // Walk engine left and right feet position
  struct Foot left, right;
  rhoban_utils::PolySpline swingSpline;

  // Support and flying foot
  bool isLeftSupport;
  struct Foot& supportFoot();
  struct Foot& flyingFoot();

  bool enableCircular;

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
  double footYOffsetPerStepSizeY;
  double trunkPitch;

  // Dynamics orders
  double stepSizeX, stepSizeY, stepSizeYaw;

  // Duration of the current step
  double stepDuration;

  // This is stored for the current step to avoid having it changing during the step itself
  double _swingGain;
};
}  // namespace rhoban