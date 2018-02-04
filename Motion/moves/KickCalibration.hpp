#pragma once

#include "Move.h"

class Approach;
class KickController;
class KickCalibration : public Move
{
public:
  KickCalibration(Approach *approach);
  std::string getName();
  
  void onStart();
  void onStop();
  void step(float elapsed);

  void toFile(std::map<std::string, std::map<std::string,  float>> &map, std::string filename);
  
protected:

  float startX;
  float T;
  bool forceStart;
  bool started;
  Approach *approach;

  std::string lastState;
  std::string currentState;

  float ballDistance;
  float ballX;
  float ballY;

  bool logging;

  std::map<std::string, std::map<std::string, float>> ballMap;

  int nbShoots;

  double tLogging;
  KickController * kickController;

  bool smallKick;
  bool normalKick;
  bool lateralKick;

  double kickCap;
};
