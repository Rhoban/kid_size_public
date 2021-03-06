#pragma once

#include "moves/KickController.h"
#include "ApproachMove.h"
#include <strategy/KickStrategy.hpp>

class Head;
class Walk;
class Kick;

class TCJump : public Move
{
public:
  TCJump(Walk* walk, Head* head, Kick* kick);

  /// Implement Move
  virtual std::string getName() override;
  virtual void onStart() override;
  virtual void onStop() override;
  virtual void step(float elapsed) override;

  bool isRunning;
  double t;

  Walk* walk;
  Head* head;
  Kick* kick;
};
