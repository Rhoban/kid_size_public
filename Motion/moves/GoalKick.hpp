#pragma once

#include "Move.h"
#include "rhoban_geometry/point.h"
#include <Eigen/Dense>

class Approach;
class KickController;
class GoalKick : public Move
{
public:
  
  GoalKick();
  std::string getName();
  
  void onStart();
  void onStop();
  void step(float elapsed);

  // void toFile(std::map<std::string, std::map<std::string,  float>> &map, std::string filename);
  Eigen::Vector2d getBallDir(rhoban_geometry::Point previousBallPos,
                             rhoban_geometry::Point currentBallPos);
  
protected:

  rhoban_geometry::Point previousBallPosition;
  float previousBallDistance;
  
  rhoban_geometry::Point currentBallPosition; 
  float currentBallDistance;

  bool ballGettingCloser;
};
