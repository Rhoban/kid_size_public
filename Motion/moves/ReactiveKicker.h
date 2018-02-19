#pragma once

#include "ApproachMove.h"

class Walk;

/// This class simply launches a kick if the ball is estimated to be
/// inside the kick zone
class ReactiveKicker : public ApproachMove {
public:
  ReactiveKicker(Walk * walk);
  
  /// Implement Move
  virtual std::string getName() override;
  virtual void onStart() override;
  virtual void step(float elapsed) override;

private:
  /// Time that should be anticipated for [s]
  double anticipation;

  /// Time spent in current phasis [s]
  double time;

  /// Is the robot kicking actually
  bool is_kicking;
};
