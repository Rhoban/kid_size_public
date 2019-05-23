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
  
  // void bufferedSetState(const std::string&);
    //  void setTeamPlayState(rhoban_team_play::TeamPlayState state);

  float t;
  //  float targetX, targetY;
  float homeX, homeY;
  float xAttack, yAttack;
  float xAttackHys, yAttackHys;
  float xIgnoreBall;
  float distanceAttack;
  float xApprox, yApprox;
  rhoban_geometry::Point needed_pos;
  double needed_angle;
  float timeSinceStop;
  int ySign;
  float coeffa;
  float coeffb;
  float yPos;
  float distX, distY;
  //init values
  /*  float init_left_hip_pitch;
 float init_right_hip_pitch;
  float init_left_knee;
  float init_right_knee;
  float ini_left_ankle_pitch;
  float init_right_ankle_pitch;
  float init_left_shoulder_roll;
  float init_right_shoulder_roll;
  float init_left_elbow;
  float init_right_elbow;*/
  
  // std::vector<std::string> nextState;
  //int nextStateIndice;
  // bool isPlacing;
  // bool neverWalked;
  bool placedByHand;
};
