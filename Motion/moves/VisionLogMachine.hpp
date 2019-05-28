#pragma once

#include "STM.h"

#include <random>

class Head;
class Walk;
class Placer;

/**
 * The VisionLogMachine move can be used to make sure a robot alternate between moving phases and scanning phase, in
 * order to acquire data. Target poses are generated randomly in a limited space.
 */
class VisionLogMachine : public STM
{
public:
  VisionLogMachine(Walk* walk, Head* head, Placer* placer);

  virtual std::string getName() override;
  virtual void onStart() override;
  virtual void onStop() override;
  virtual void step(float elapsed) override;

  virtual void enterState(std::string state) override;
  virtual void exitState(std::string state) override;

  /**
   * Uses current pose of the robot to generate the next placer target while ensuring constraints are respected
   */
  void generatePlacerOrder();

private:
  Walk* walk;
  Head* head;
  Placer* placer;

  std::default_random_engine engine;

  /**
   * Boundaries for the search area
   */
  float minX, maxX, minY, maxY;

  /**
   * Minimal distance traveled between two acquisition points [m]
   */
  float minDistance;
  /**
   * Maximal distance traveled between two acquisition points [m]
   */
  float maxDistance;

  /**
   * Time spent in current state
   */
  float stateTime;
  /**
   * Time elapsed since beginning of the run in seconds
   */
  float runTime;
  /**
   * Duration of a run in seconds
   */
  float runDuration;
};
