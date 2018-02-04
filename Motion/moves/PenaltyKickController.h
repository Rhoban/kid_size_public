#pragma once

#include "KickController.h"

class Walk;

class PenaltyKickController : public KickController
{
public:
  PenaltyKickController();
  std::string getName();

  void onStart();
  void step(float elapsed);

  void setLeft(bool left);

protected:
  bool left;
};
