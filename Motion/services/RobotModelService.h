#pragma once

#include "Service.h"
#include "robot_model/humanoid_model.h"

class Move;

class RobotModelService : public Service
{
public:
  /**
   * Initialization
   */
  RobotModelService();

  bool tick(double elapsed);

  // Models
  rhoban::HumanoidModel goalModel;
  rhoban::HumanoidModel readModel;
};
