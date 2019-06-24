#pragma once

#include "ApproachMove.h"

class Walk;
class Head;

/// This class simply launches a kick if the ball is estimated to be
/// inside the kick zone
class ReactiveKicker : public ApproachMove
{
public:
  ReactiveKicker(Walk* walk, Kick* kick, Head* head);

  /// Implement Move
  virtual std::string getName() override;
  virtual void onStart() override;
  virtual void onStop() override;
  virtual void step(float elapsed) override;

  virtual rhoban_utils::Angle getKickCap() override;

private:
  Head* head;

  /// Time that should be anticipated for [s]
  double anticipationMean;
  double anticipationDelta;

  /// Time where the kick should pause [s]
  double kickPauseTime;

  /// Is the robot kicking actually
  bool is_kicking;

  /// Left foot ?
  bool useRightFoot;
};
