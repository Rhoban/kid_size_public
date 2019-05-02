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

  void stepArms();

protected:
  // Walk engine
  rhoban::WalkEngine engine;

  // Walking parameters
  double walkStep, walkLateral, walkTurn;

  // Walking acc. limits
  double maxDStepByCycle, maxDLatByCycle, maxDTurnByCycle;

  // Walk state
  enum WalkState {
    WalkNotWalking = 0,
    WalkStarting,
    Walking,
    WalkStopping
  };

  WalkState state;

  // Step count since walk enabled
  int stepCount = 0;

  // Time lapsed since last step
  double timeSinceLastStep;

  // Control flag to enable or disable the walk
  bool walkEnable;

  // Is the walk actually enabled now ?
  bool walkEnabled;

  // Was the walk enabled on last tick ?
  bool walkWasEnabled;

  // Extra trunk pitch
  double trunkPitch;

  // Swing gain on starting steps
  double swingGainStart;

  // Arms parameters
  double armsRoll, elbowOffset;
};
