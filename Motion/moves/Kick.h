#pragma once

#include <rhoban_utils/spline/function.h>
#include "Move.h"

#include "kick_model/kick_model_collection.h"

class Walk;
class Head;
class Arms;

class Kick : public Move
{
public:
  Kick(Head* head_, Walk* walk_, Arms* arms);

  std::string getName();

  void onStart();
  void onStop();

  void stepSpline(float elapsed);

  void step(float elapsed);
  bool over;
  bool live;
  std::string cmdKickGen();

  void loadKick(std::string filename);

  void set(bool left, const std::string& kickName, bool pause = false, double pauseTime = 0.0);
  void unpause();

  static std::string getPath(const std::string kickName, bool left = false);

protected:
  typedef std::map<std::string, rhoban_utils::Function> Splines;

  bool useManualT;
  double manualT;
  bool applied;
  double t, tMax;
  double dt;
  double applyKickRatio;

  bool pause;
  double pauseTime;

  // Warmup and cooldown
  double warmup;
  double cooldown;

  enum KickState
  {
    KickNotKicking = 0,
    KickWaitingWalkToStop,
    KickWarmup,
    KickKicking,
    KickCooldown
  };

  KickState kickState;

  // Kick mode
  bool left, generated;

  // Name of the kick which should be used
  std::string kickName;

  // Splines
  std::map<std::string, rhoban_utils::Function> splines;

  // Preloaded splines
  std::map<std::string, Splines> preloadedSplines;
  std::map<std::string, double> maxTimes;

  // The collection of available kicks
  csa_mdp::KickModelCollection kmc;

  void apply();

  Head* head;
  Walk* walk;
  Arms* arms;

  bool headWasDisabled;
};
