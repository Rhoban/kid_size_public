#pragma once

#include "Move.h"
#include "engines/walk_engine.h"

class Kick;
class Walk : public Move
{
public:
  Walk(Kick*);
  std::string getName();

  void onStart();
  void onStop();
  void step(float elapsed);

  // Control the robot using [mm] and [deg]
  void control(bool enable, double step = 0, double lateral = 0, double turn = 0);

  /**
   * Boundaries for orders and deltaOrders (step, lateral, turn)
   * units are: [m/step], [rad/step], [m/step^2] and [rad/step^2]
   */
  Eigen::Vector3d getMinOrders() const;
  Eigen::Vector3d getMaxOrders() const;
  Eigen::Vector3d getMinDeltaOrders() const;
  Eigen::Vector3d getMaxDeltaOrders() const;

  /**
   * Return the current raw walk goal displacement
   * dX, dY, dTheta, enabledGain.
   */
  Eigen::Vector4d getRawOrder() const;

  /**
   * Return last command applied to walk (before trim)
   */
  Eigen::Vector4d getOrder() const;

  /**
   * Set raw (meters, radian) current walk
   * goal displacement: dX, dY, dTheta, enabled
   */
  void setRawOrder(double step, double lateral, double turn, bool enable);
  /**
   * params: step[m], lat[m], rot [rad]
   */
  void setRawOrder(const Eigen::Vector3d& params, bool enabled);

  // Starts a kick
  void kick(bool rightFoot, const std::string& kickName);

  // Is the walk currently kicking ?
  bool isKicking();

  // Will the new step be a new step ?
  bool isNewStep(double elapsed);

  // Gets the walk phase, 0: begin of cycle, 0.5: step 1, 1: step2 
  double getPhase();

  // Maximum rotation speed [deg/step]
  float maxRotation;

  // Maximum stepping [mm/step]
  float maxStep, maxStepBackward;

  // Maximum lateral [mm/step]
  float maxLateral;

protected:
  // Walk engine
  rhoban::WalkEngine engine;

  // Walking parameters
  double walkStep, walkLateral, walkTurn;

  // Walking acc. limits
  double maxDStepByCycle, maxDLatByCycle, maxDTurnByCycle;

  // Walk state
  enum WalkState
  {
    WalkNotWalking = 0,
    WalkStarting,
    Walking,
    WalkStopping
  };

  WalkState state;

  // Step count since walk enabled
  int stepCount = 0;

  // Time lapsed since last step
  double timeSinceLastStep;

  // Control flag to enable or disable the walk
  bool walkEnable;

  // Extra trunk pitch
  double trunkPitch;

  // Swing gain on starting steps
  double swingGainStart;

  // Arms parameters
  double armsRoll, elbowOffset;
  void stepArms();

  // Security parameters
  double securityThreshold;
  double securityPhase;

  // Kicking
  Kick* kickMove;
  bool kickPending;
  bool kickLeftFoot;
  double kickWarmup;
  double kickCooldown;
  double kickT;
  std::string kickName;

  enum KickState
  {
    KickNotKicking = 0,
    KickWaitingWalkToStop,
    KickWarmup,
    KickKicking,
    KickCooldown
  };

  KickState kickState;
  void stepKick(float elapsed);
};
