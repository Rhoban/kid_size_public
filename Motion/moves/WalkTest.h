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

protected:
  double walkT;
  rhoban::WalkEngine engine;
};
