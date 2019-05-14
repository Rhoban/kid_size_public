#include "walk_engine.h"
#include "robot_model/humanoid_model.h"
#include "rhoban_utils/angle.h"

using namespace rhoban_utils;

/**
 * This binary just displays some outputs of the walk engine for plotting and debugging
 * purpose.
 */
int main()
{
  // Initializing walk engine with humanoid model
  rhoban::HumanoidModel model;
  rhoban::WalkEngine engine;
  engine.initByModel(model);

  engine.stepSizeYaw = 1;
  engine.trunkPitch = 0.6;
  engine.newStep();

  double totalT = 0;
  for (int step = 0; step < 4; step++)
  {
    engine.newStep();
    for (double t = 0; t < engine.stepDuration; t += 0.001)
    {
      totalT += 0.001;
      rhoban::WalkEngine::FootPose pose = engine.left.getPosition(t);
      std::cout << totalT << " " << pose.x << " " << pose.y << " " << pose.z << " " << pose.yaw << std::endl;
    }
  }
}