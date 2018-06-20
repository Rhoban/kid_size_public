#pragma once

#include "KickController.h"

class Walk;

class ClearingKickController : public KickController
{
  bool isInLateral=false;
public:
  ClearingKickController();
  std::string getName();

  void step(float elapsed);
};
