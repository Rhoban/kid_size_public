#pragma once

#include "moves/KickController.h"
#include "ApproachMove.h"
#include <strategy/KickStrategy.hpp>

class Head;
class Walk;

class TCMovingBallPasser : public KickController
{
public:
  TCMovingBallPasser(Walk* walk, Head* head);

  /// Implement Move
  virtual std::string getName() override;
  virtual void onStart() override;
  virtual void onStop() override;
  virtual void step(float elapsed) override;

  KickStrategy strategy;

  bool isRunning;
  double t;
  double kickDirection;

  Walk* walk;
  Head* head;
};
