#pragma once

#include "moves/KickController.h"
#include "ApproachMove.h"
#include <strategy/KickStrategy.hpp>

class Head;
class Walk;

/**
 * Dirty diving penalty keeper, hack the walk when launched
 */
class PenaltyKeeper : public Move
{
public:
  PenaltyKeeper(Walk* walk, Head* head);

  /// Implement Move
  virtual std::string getName() override;
  virtual void onStart() override;
  virtual void onStop() override;
  virtual void step(float elapsed) override;

  void setShouldersTorque(bool enabled);

  bool check_only;

  double t;

  bool is_running;
  bool is_diving;
  bool dive_left;

  double min_dive_y;

  double anticipation;
  double em_start;

  Walk* walk;
  Head* head;
};
