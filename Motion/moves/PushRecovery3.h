#pragma once

#include "Move.h"

#include <json/json.h>
#include <Eigen/Dense>
#include <deque>

class Walk;
class PushRecovery3 : public Move
{
  
public:
  
  PushRecovery3(Walk *walk);
  
  void onStart();
  void onStop();
  std::string getName();
  void step(float elapsed);

  Eigen::Vector3d nextStepPosition();
  
protected:
  
  Walk *walk;
  
  bool pushWalkEnable; 
  bool enableLog;
  bool saved;
  bool loaded;
  bool waiting;
  bool cooldown;
  bool ready;
  bool recoverFall;
  bool triggerLeftFoot;
  bool triggerRightFoot;
  
  double logTime;
  double t;
  double thresholdNominalScore;
  double coeffStepSize;
  double cooldownTime;

  double leftTimeSinceTouchedGround;
  double rightTimeSinceTouchedGround;

  double angleHip;
  double angleAnkle;

  double offsetHip;
  double offsetAnkle;

  double tMove;
  double perturbationLastIteration;// To avoid outliers

  bool enableReaction;
  
  std::deque<double> meanAccXQueue;

  double thresholdMeanAccX;

  double normalTrunkPitchBackwards;

  double offsetTrunkPitch;

  double offsetTrunkZ;
  
};
