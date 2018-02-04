#pragma once

#include <rhoban_utils/spline/function.h>
#include <Model/HumanoidFloatingModel.hpp>
#include <Model/InverseKinematics.hpp>
#include "Move.h"

class LateralStep : public Move
{
public:
  LateralStep();
  std::string getName();

  void onStop();
  void onStart();
  void step(float elapsed);
  bool live;
  std::string cmdLateralStepGen();
  std::map<std::string, rhoban_utils::Function> loadCompiledLateralStep(std::string filename);
  void loadCompiledLateralSteps();
  void loadLateralStep(std::string filename);
  void setLeft(bool left);

protected:
  bool applied;
  double t, tMax;
  double dt;
  double applyLateralStepRatio;
  bool over;

  bool  left,generated;

  // This floating model is used for inverse kinematics
  Leph::HumanoidFloatingModel lateralStepModel;
  // Inverse kinematics
  Leph::InverseKinematics inv;

  // Splines
  std::map<std::string, rhoban_utils::Function> splines;

  // Preloaded splines
  double tMaxRegular, tMaxLateral, tMaxSmall;

  void apply();
};
