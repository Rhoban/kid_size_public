#pragma once

#include "moves/KickController.h"
#include "ApproachMove.h"
#include <strategy/KickStrategy.hpp>

class Head;
class Walk;

class TCReactiveKicker : public Move
{
public:
  TCReactiveKicker(Walk* walk, Head* head);

  /// Implement Move
  virtual std::string getName() override;
  virtual void onStart() override;
  virtual void onStop() override;
  virtual void step(float elapsed) override;

  KickStrategy strategy;

  bool yPositive;
  bool isRunning;
  double t;

  Walk* walk;
  Head* head;
};
