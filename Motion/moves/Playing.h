#pragma once

#include <rhoban_geometry/point.h>
#include <services/TeamPlayService.h>
#include "STM.h"

class Walk;
class Head;
class Approach;
class Placer;
class PlayingMove : public STM
{
public:
  struct Place {
    bool ok;
    rhoban_geometry::Point position;
    rhoban_utils::Angle orientation;
  };

  PlayingMove(Walk *walk);
  std::string getName();

  void onStart();
  void onStop();
  void step(float elapsed);
        
  virtual void enterState(std::string state);
  virtual void exitState(std::string state);
  void setTeamPlayState(rhoban_team_play::TeamPlayState state);

protected:
  double t;
  double backwardT;
  double localizeWalkDuration;

  bool useKickController;
  bool stopOnHandle;
  bool teamConfidence;
  double avoidRadius;
  double walkBallDistance;

  Walk *walk;
  Head *head;
  Approach *approach;
  Placer *placer;
};
