#pragma once

#include "STM.h"

#include <rhoban_geometry/point.h>

class Walk;
class Approach;
class Placer;

class GoalKeeper : public STM
{
public:
  GoalKeeper(Walk* walk, Placer* placer);
  std::string getName();

  void onStart();
  void onStop();
  void step(float elapsed);
  void enterState(std::string state);
  void exitState(std::string state);

protected:
  Walk* walk;
  Placer* placer;

  rhoban_geometry::Point home();

  bool ballInZone(float xd, float yd);
  bool ballInDangerZone();
  bool ballInAttackZone();
  bool ballInAttackHysZone();
  bool isBallSafe();
  rhoban_geometry::Point alignBallPos();
  float getAngle();
  bool goodEnoughPos(rhoban_geometry::Point pos, rhoban_geometry::Point needed_pos);

  float t;
  float homeX, homeY;
  float xAttack, yAttack;
  float xAttackHys, yAttackHys;
  float xIgnoreBall;
  float distanceAttack;
  float xApprox, yApprox;
  rhoban_geometry::Point needed_pos;
  double needed_angle;

  int ySign;
  float coeffa;
  float coeffb;
  float yPos;
  float distX, distY;

  bool placedByHand;
};
