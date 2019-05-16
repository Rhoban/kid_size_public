#pragma once

#include "Move.h"
#include <rhoban_utils/spline/function.h>
#include <iostream>
#include <string>

class ThrowIn : public Move
{
 public:
  ThrowIn();
  ~ThrowIn();
  std::string getName();
  
  void onStart();
  void onStop();
  void step(float elapsed);

  bool over;
  int trying;
  
  
 protected:
  float time;
  float tMax;

  double manualT;
  bool reloadSpline;
bool useManualT;
  
  std::map<std::string, rhoban_utils::Function> splines;

};
