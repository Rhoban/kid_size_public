#pragma once

#include "moves/KickController.h"
#include "ApproachMove.h"
#include <strategy/KickStrategy.hpp>

class Head;
class Walk;

class TCHighKick : public KickController
{
public:
  TCHighKick(Walk* walk, Head* head);

  /// Implement Move
  virtual std::string getName() override;
  virtual void onStart() override;
  virtual void onStop() override;
  virtual void step(float elapsed) override;

  KickStrategy strategy;

  bool isRunning;
  double t;

  Walk* walk;
  Head* head;
};
