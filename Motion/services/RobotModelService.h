#pragma once

#include "Service.h"
#include "robot_model/humanoid_model.h"
#include "robot_model/humanoid_server.h"

class Move;

class RobotModelService : public Service
{
public:
  /**
   * Initialization
   */
  RobotModelService();

  bool tick(double elapsed);

  // Should we update the base ?
  void enableOdometry(bool enabled);
  bool odometryEnabled;

  // Models
  rhoban::HumanoidModel model;
  rhoban::HumanoidServer server;

  double timeSinceLastPublish;
};
