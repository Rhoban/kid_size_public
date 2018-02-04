#pragma once

#include <vector>
#include <random>
#include <string>
#include <mutex>
#include <Utils/RandomWalk.hpp>
#include <Eigen/Core>
#include <Model/HumanoidFixedModel.hpp>
#include <Odometry/OdometrySequence.hpp>
#include "STM.h"

class Head;
class Walk;

/// LogMachine motion allows to retrieve walk logs, it is highly based on
/// OdometryCalibration
class LogMachine : public STM
{
public:
  LogMachine(Walk* walk, Head * head);

  virtual std::string getName() override;
  virtual void onStart() override;
  virtual void step(float elapsed) override;

  virtual void enterState(std::string state) override;
  virtual void exitState(std::string state) override;

private:
  /// Start a new run and open a sequence
  void startRun();

  /// End current log
  void endRun();

  /// Remove last log entered
  void removeLastLog();

  /// Save current status into a hidden file for backup
  void safetyBackup();

  std::string cmdSetLastRunPos(double dx, double dy, double da);
  std::string cmdStartLog();
  std::string cmdStartSequence(int nbRuns);
  std::string cmdStopLog();
  std::string cmdCancelLog();
  std::string cmdDumpData(std::string filename);
  std::string cmdClear();

  Eigen::Vector3d getPosition();

  Eigen::Matrix<double,3,2> getOrdersLimits();
  Eigen::Matrix<double,3,2> getDeltaLimits();

  Eigen::Vector3d getRandomSpeed();

  std::default_random_engine engine;

  /// Access to walk motion
  Walk* walk;

  /// Access to Head move
  Head * head;

  /// Brief summary of status for RhIO inspection
  std::string status;

  /// Sequences stored in motion
  std::vector<Leph::OdometrySequence> sequences;

  /// Access to the sequences need to be stored safely
  std::mutex sequencesMutex;

  /// Dump all sequences to dumpFileName during step if it is not empty
  std::string dumpFileName;

  /// Automatic random exploration
  Leph::RandomWalk randomWalk;

  /// Is the log acquisition autonomous? (walking+localizing)
  bool autoMode;

  /// Are the walk orders drawn from random distributions or are they chosen
  /// manually
  bool customOrders;

  /// Which dimensions are allowed to have non-zero values for randomWalks
  /// -1: all
  /// 0+: only dimension 'selectedDim'
  int selectedDim;

  /// Boundaries on the signs allowed for exploration
  /// -: only negative values
  /// 0: both positive and negative values
  /// +: only positive values
  int selectedSign;

  /// When orders are not custom, is the speed chosen at the beginning of the
  /// run or is it fixed?
  bool fixedSpeed;

  /// Selected speed when using random orders
  Eigen::Vector3d autoSpeed;

  /// Custom command for Step
  double cmdStep;
  /// Custom command for Step
  double cmdLat;
  /// Custom command for Step
  double cmdRot;

  /// If > 0, then represent the number of cycles used for autoWalk
  int autoNbSteps;

  /// Number of steps elapsed in current log
  int stepCount;

  /// Time elapsed since the beginning of the phase [s]
  double time;

  /// Position at the beginning of current log
  Eigen::Vector3d initialPosition;

  /// Time allowed for auto-localize [s]
  double localizeDuration;

  /// Number of auto logs remaining
  /// TODO: currently do not handle standup on falls and similar stuff
  int nbRunsLeft;
};

