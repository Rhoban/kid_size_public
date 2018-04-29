#pragma once

#include <rhoban_utils/spline/function.h>
#include "Move.h"

#include "kick_model/kick_model_collection.h"

class Kick : public Move
{
public:
  Kick();
  std::string getName();

  void onStart();
  void step(float elapsed);
  bool over;
  bool live;
  std::string cmdKickGen();
  std::map<std::string, rhoban_utils::Function> loadCompiledKick(std::string filename);
  void loadCompiledKicks();
  void loadKick(std::string filename);

  void set(bool left, const std::string & kickName);

  static std::string getPath(const std::string kickName, bool left=false);
  static std::string getCompiledPath(const std::string kickName, bool left=false);
protected:

  typedef std::map<std::string, rhoban_utils::Function> Splines;

  bool useManualT;
  double manualT;
  bool applied;
  double t, tMax;
  double dt;
  double applyKickRatio;
        
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
};
