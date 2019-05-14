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

  /**
   * Send target motor position to RhAL low level.
   * Applay all DOF of only apply both arms,
   * both legs, head or left/right leg.
   */
  void flushAll(double gain = 1.0);
  void flushArms(double gain = 1.0);
  void flushHead(double gain = 1.0);
  void flushLegs(double gain = 1.0);
  void flushLeftLeg(double gain = 1.0);
  void flushRightLeg(double gain = 1.0);
  void flush(bool doHead, bool doLeftArm, bool doRightArm, bool doLeftLeg, bool doRightLeg, double gain = 1.0);

  // Models
  rhoban::HumanoidModel goalModel;
  rhoban::HumanoidModel readModel;
};
