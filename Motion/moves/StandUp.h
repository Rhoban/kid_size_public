#pragma once

#include "Move.h"
#include <rhoban_utils/spline/function.h>
#include <iostream>
#include <string>

class Walk;
class StandUp : public Move
{
public:
  StandUp();
  ~StandUp();
  std::string getName();

  void onStart();
  void onStop();
  void step(float elapsed);
  void setLayDown(bool value);

  bool over;
  int trying;

protected:
  float armsRoll;
  float time;
  float speed_front;
  float speed_back;
  float speed;
  std::map<std::string, rhoban_utils::Function> splines;
  bool enable, waiting;
  bool useManualT;
  bool layDown;  // If true we use the give up motion
  double layDownEnd;
  double manualT;
  double delayBefore;

  std::string currentSpline;
  bool reloadSpline;
};
