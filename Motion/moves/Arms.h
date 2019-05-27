#pragma once

#include "Move.h"

class Arms : public Move
{
public:
  enum ArmsState : int
  {
    ArmsDisabled = 0,
    ArmsEnabled = 1,
    ArmsMaintenance = 2
  };

  struct armsAngle
  {
    double elbow;
    double shoulder_pitch;
    double shoulder_roll;
  };

  Arms();
  std::string getName();

  void onStart();
  void onStop();
  void step(float elapsed);

  // Enabling/disabling arms
  // force = true should only be used internally
  void setArms(ArmsState armsState, bool force = false, bool init = false);

  ArmsState armsState;
  armsAngle actualAngle;
  armsAngle lastAngle;

protected:
  // Arms parameters
  double armsRoll, maintenanceArmsRoll, disabledArmsRoll;
  double elbowOffset, maintenanceElbowOffset, disabledElbowOffset;
  double smoothingArms;
  bool armsEnabled;
  bool maintenanceArmsEnabled;

  void stepArms(double elapsed);
};
