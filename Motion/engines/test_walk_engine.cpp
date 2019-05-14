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

  engine.stepSizeX = 0.1;
  engine.newStep();

  for (int step = 0; step < 2; step++)
  {
    engine.newStep();
    for (double t = 0; t < engine.stepDuration; t += 0.01)
    {
      rhoban::WalkEngine::FootPose pose = engine.left.getPosition(t);
      std::cout << t << " " << pose.x << " " << pose.y << " " << pose.z << " " << pose.yaw << std::endl;
    }
  }
}