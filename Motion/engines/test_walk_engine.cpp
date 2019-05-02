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

  ///XXX
}