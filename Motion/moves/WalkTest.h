#pragma once

#include "Move.h"
#include "engines/walk_engine.h"

class WalkTest : public Move
{
public:
  WalkTest();
  std::string getName();

  void onStart();
  void step(float elapsed);

  std::string cmdOneStep(double dx, double dy, double dtheta);

protected:
  double walkT;

  bool walkEnable;
  double trunkPitch;
  rhoban::WalkEngine engine;

  bool askOneStep;
  double oneStepX, oneStepY, oneStepTheta;

  double armsRoll, elbowOffset;
  int stepCount = 0;
};
