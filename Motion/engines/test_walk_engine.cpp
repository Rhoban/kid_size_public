#include "walk_engine.h"
#include "Model/HumanoidFixedModel.hpp"
#include "rhoban_utils/angle.h"

using namespace rhoban_utils;

int main()
{
  // Initializing walk engine with humanoid model
  Leph::HumanoidFixedModel model(Leph::RobotType::SigmabanModel);
  rhoban::WalkEngine engine;
  engine.initByModel(model);

  // engine.assignModel(model);
  engine.xSpeed = 0.1;
  engine.yawSpeed = 0.1;
  engine.reset();

  double t = 0;

  for (int k=0; k<400; k++) {
    t += 0.01;
    double overTime = engine.update(t);
    if (overTime > 0) {
      t = overTime;
      engine.newStep();
      engine.update(t);
    }

    auto pose = engine.right.getPosition(engine._t);
    std::cout << engine._t << " " << pose.x << " " << pose.y << " " << pose.z << " " << pose.yaw << " ";
    pose = engine.left.getPosition(engine._t);
    std::cout << pose.x << " " << pose.y << " " << pose.z << " " << pose.yaw << std::endl;
    usleep(30000);
  }
}