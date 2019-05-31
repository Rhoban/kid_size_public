#pragma once

#include "STM.h"
#include <string>
#include <services/TeamPlayService.h>

class Placer;
class StandUp;
class Walk;
class Arms;

/**
 * Test cases:
 *
 * - When entering initial phase, the filters should be reset on auto kick off position
 * - When starting robocup during ready phase (placing), the robot should not start to walk and will apply
 *   border reset when playing
 * - When entering set phase, robots should stop to move
 * - If a robot is handled during this set phase, it should after do a border reset
 * - If the robot is handled while playing normally, nothing should happen
 * - If a robot is penalized, and then handled it should do a border reset at the begining of its timer,
 *   and no reset at the end of timer
 * - If a robot is penalized and not touched, we assume it's a mistake and we don't do any reset
 * - If a robot is penalized and is put on the floor after the timer started, it should border-resets its filter
 *   at this time
 * - If a robot is serving a penalty and is handled, border reset should be done again
 * - After a goal is scored, robots that are handled during placement phase or freeze phase should apply a border reset
 * - When a robot is penalized (or substitute for eg., or waiting to play, the head should not be scanning)
 */
class Robocup : public STM
{
public:
  Robocup(Walk* walk, StandUp* standup, Placer* placer, Arms* arms);
  std::string getName();
  bool standup_disabled;
  void onStart();
  void onStop();
  void step(float elapsed);
  bool isFallen();

  bool isGoalKeeper() const;

protected:
  Walk* walk;
  StandUp* standup;
  Placer* placer;
  Arms* arms;

  float t;
  bool isHandled;
  bool wasHandled;
  bool lastRefereePlaying;
  bool rememberStart;
  bool goalKeeper;
  bool freeKicker;
  float freeKickerExtraX;
  bool autoKickOff;
  int standup_try;
  double timeSinceVisionInactive;
  double timeSinceNoConsistency;

  // Robot position at end of initial phase for autoKickOff
  float autoStartX;
  float autoStartY;
  float autoStartAzimuth;
  // Target of the robot for autoKickOff when not being freeKicker
  float autoTargetX;
  float autoTargetY;
  // Target of the robot for autoKickOff when freeKicker
  float freeKickX;
  float freeKickY;

  void applyGameState();

  void enterState(std::string state);
  void exitState(std::string state);
};
