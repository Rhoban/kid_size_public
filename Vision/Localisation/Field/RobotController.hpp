#pragma once

#include "FieldPosition.hpp"

#include "rhoban_unsorted/particle_filter/controller.h"

namespace Vision
{
namespace Localisation
{
class RobotController : public rhoban_unsorted::Controller<FieldPosition>
{
private:
  static double posExploration, angleExploration;
  static Eigen::MatrixXd posLimits;

  static int nbSamples;

  rhoban_geometry::Point ctrlMove;
  rhoban_utils::Angle ctrlRot;

  double noiseGain;

public:
  RobotController();
  // Move is the expected move of the robot in the robot referential, not the
  // speed (changed july 19th 2015)
  RobotController(const rhoban_geometry::Point& move, const rhoban_utils::Angle& rotation, double noiseGain);

  static void setPosExploration(double newPosExplo);
  static double getPosExploration()
  {
    return posExploration;
  }
  static double getAngleExploration()
  {
    return angleExploration;
  }

  virtual void move(FieldPosition& p, double elapsedTime) override;
  virtual void explore(FieldPosition& p, double elapsedTime) override;

  static void bindWithRhIO();
  static void importFromRhIO();
};
}  // namespace Localisation
}  // namespace Vision
