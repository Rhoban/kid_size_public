#pragma once

#include <map>
#include <string>
#include <rhoban_utils/spline/function.h>
#include "Move.h"
#include <Model/HumanoidFloatingModel.hpp>
#include <Model/HumanoidFixedModel.hpp>
#include <Types/MatrixLabel.hpp>
#include <IKWalk/IKWalk.hpp>
#include <Utils/Scheduling.hpp>

class StaticLearner : public Move
{
public:
  StaticLearner();
  std::string getName();
  
  void onStart();
  void onStop();
  
  void step(float elapsed);

  bool _staticOn;
  bool _staticOnArms;
  bool _staticOnLeftLeg;
  bool _staticOnRightLeg;
  float _pLegs;
  float _pArms;
  float _iLegs;
  float _iArms;
  int _staticPointIndex;
  bool _snapshot;
  bool _goToOn;
  bool _goToIndex;
private:

  void savePositions(int position=-1);

  void followPresentPositions();
  void followPresentPositionsArms();
  void followPresentPositionsLeftLeg();
  void followPresentPositionsRightLeg();
  
  void setPLegs();

  void setPArms();
  void savePositionsToFile(std::string fileName, std::map<std::string, std::vector<float>>& map);
  void savePositionsToFileJson(std::string fileName, std::map<std::string, std::vector<float>>& map);
  void readPositionsFromFile(std::string fileName, std::map<std::string, std::vector<float>>& map);
  void calculateExtraAngles();
  void addExtraAngles();
  void goToPositions(int index);
  
  int _oldPointIndex;
  bool _oldStaticOn;
  
  // Name helpers
  std::vector<std::string> _dofNames;
  std::vector<std::string> _legsDofNames;
  std::vector<std::string> _leftLegDofNames;
  std::vector<std::string> _rightLegDofNames;
  std::vector<std::string> _armsDofNames;

  // Map of dofName -> list of positions for that DOF
  std::map<std::string, std::vector<float>> _mapOfPositions;

  // Map of dofName -> I value
  std::map<std::string, float> _mapOfIs;

  // Map of dofName -> extra angle
  std::map<std::string, float> _mapOfExtraAngles;

  // Map of dofName -> actual goal position
  std::map<std::string, float> _mapOfActualGoalPosition;

};
