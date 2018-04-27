#pragma once

#include "STM.h"

#include <services/TeamPlayService.h>
#include <rhoban_geometry/point.h>

class Walk;
class Approach;
class Placer;

class GoalKeeper : public STM
{
public:
  GoalKeeper(Walk *walk,  Placer *placer);
  std::string getName();

  void onStart();
  void onStop();
  void step(float elapsed);
  void enterState(std::string state);
  void exitState(std::string state);
  void setTeamPlayPriority(rhoban_team_play::TeamPlayPriority priority);
  rhoban_team_play::TeamPlayState teamState();

protected:
  Walk *walk;
  Placer *placer;

	
  bool ballInZone(float xd, float yd);
	bool ballInAttackZone();
	bool ballInAttackZoneHysteresis();
	bool ignoreBall();
	bool ignoreBallHys();
	bool isNearHome();
	bool isNearHomeHys();
	rhoban_geometry::Point getAlignPoint(const rhoban_geometry::Point &,float &);
	rhoban_geometry::Point home();
	bool isAligned();
  rhoban_geometry::Point shootLineCenter();

	void bufferedSetState(const std::string &);
	
  /// pos[m] and azimuth[deg] are output parameters
  void getFieldTarget(rhoban_geometry::Point & wishedPos, rhoban_utils::Angle & wishedAzimuth);
  void setTeamPlayState(rhoban_team_play::TeamPlayState state);
        
  float t;
  float targetX, targetY, targetSmoothing;
  float xAttack, yAttack;
  float xAttackHys, yAttackHys;
  float xIgnoreBall, xIgnoreBallHys;
  float homeX,maxHomeDistance,maxHomeDistanceHys;
	float alignTolerance;
	float maxShootDist;
	int nextStateSize;
	std::vector<std::string> nextState;
	int nextStateIndice;	
  //bool isPlacing;
	bool neverWalked;
	bool placedByHand;
};
