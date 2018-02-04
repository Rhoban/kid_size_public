#include <fstream>

#include <rhoban_utils/logging/logger.h>

#include "moves/LogMachine.hpp"
#include "services/DecisionService.h"
#include "services/LocalisationService.h"
#include "services/ModelService.h"
#include "Head.h"
#include "Walk.h"

#include <scheduler/MoveScheduler.h>

#include <rhoban_random/tools.h>

#define STATE_INACTIVE    "Inactive"
#define STATE_PREAMBLE    "Preamble"
#define STATE_LOGGING     "Logging"
#define STATE_EPILOGUE    "Epilogue"
#define STATE_INCAPACITED "Incapacited"

static rhoban_utils::Logger logger("LogMachine");

using namespace rhoban_utils;
using namespace rhoban_geometry;

LogMachine::LogMachine(Walk* walk_, Head * head_)
  : walk(walk_), head(head_),
    status(""),
    dumpFileName(""),
    randomWalk(3)
{
  engine = rhoban_random::getRandomEngine();
  state = STATE_INACTIVE;
  Move::initializeBinding();
  //RhIO Binding function
  bind->bindFunc("log_machine_set_last_pos", 
                 "Set target reference displacement <dX> <dY> <dA> in source frame." +
                 std::string("Assign given position to last sequence"),
                 &LogMachine::cmdSetLastRunPos, *this, {"", "", ""});
  bind->bindFunc("log_machine_start_sequence", 
                 "Start a multiple run logs sequence <nbRuns>",
                 &LogMachine::cmdStartSequence, *this);
  bind->bindFunc("log_machine_start", 
                 "Start logging a run",
                 &LogMachine::cmdStartLog, *this);
  bind->bindFunc("log_machine_stop", 
                 "Stop logging current run",
                 &LogMachine::cmdStopLog, *this);
  bind->bindFunc("log_machine_cancel", 
                 "Stop logging and pop last started learning sequence.",
                 &LogMachine::cmdCancelLog, *this);
  bind->bindFunc("log_machine_dump", 
                 "Dump recorded odometry data into given filename",
                 &LogMachine::cmdDumpData, *this);
  bind->bindFunc("log_machine_clear", 
                 "Remove all collected logs from the current list",
                 &LogMachine::cmdClear, *this);
  // RhIO binding variables
  bind->bindNew("autoNbSteps", autoNbSteps, RhIO::Bind::PullOnly)
    ->defaultValue(100)
    ->comment("Maximal number of steps when doing random exploration");
  bind->bindNew("stepCount", stepCount, RhIO::Bind::PushOnly)
    ->defaultValue(0)
    ->comment("Number of steps since beginning of random exploration");
  bind->bindNew("time", time, RhIO::Bind::PushOnly)
    ->defaultValue(0)
    ->comment("Time spent in current phase [s]?");
  // Auto localization and walk parameters
  bind->bindNew("autoMode", autoMode, RhIO::Bind::PullOnly)
    ->defaultValue(true)->persisted(true)
    ->comment("Is the robot walking and localizing autonomously");
  bind->bindNew("selectedDim", selectedDim, RhIO::Bind::PullOnly)
    ->defaultValue(-1)->persisted(true)
    ->comment("When robot is exploring randomly, which dimension should be used? (-1 = all)");
  bind->bindNew("selectedSign", selectedSign, RhIO::Bind::PullOnly)
    ->defaultValue(0)->persisted(true)
    ->comment("'-' -> only negative orders, '+' -> only positive orders, '0' -> both pos and neg");
  bind->bindNew("fixedSpeed", fixedSpeed, RhIO::Bind::PullOnly)
    ->defaultValue(false)->persisted(true)
    ->comment("Are orders fixed (true) or updated after every step (false)?");
  bind->bindNew("localizeDuration", localizeDuration, RhIO::Bind::PullOnly)
    ->defaultValue(10)->persisted(true)
    ->comment("Time allowed for localization phases [s]?");
  // Custom orders
  bind->bindNew("customOrders", customOrders, RhIO::Bind::PullOnly)
    ->defaultValue(false)->persisted(true)
    ->comment("Should cmdStep, cmdLat and cmdRot be applied?");
  bind->bindNew("cmdStep", cmdStep, RhIO::Bind::PullOnly)
    ->defaultValue(0)
    ->comment("Size of forward step [m]");
  bind->bindNew("cmdLat", cmdLat, RhIO::Bind::PullOnly)
    ->defaultValue(0)
    ->comment("Size of lateral step [m]");
  bind->bindNew("cmdRot", cmdRot, RhIO::Bind::PullOnly)
    ->defaultValue(0)
    ->comment("Rotation at each step [rad]");
  // Automatically perform a sequence of logs
  bind->bindNew("nbRunsLeft", nbRunsLeft, RhIO::Bind::PushOnly)
    ->defaultValue(0)
    ->comment("Number of runs remaining when chaining auto experiments");
}

std::string LogMachine::getName()
{
  return "LogMachine";
}
void LogMachine::onStart()
{
  sequencesMutex.lock();
  sequences.clear();
  sequencesMutex.unlock();
  dumpFileName = "";
  state = STATE_INACTIVE;
  // Ensure that walk and head are properly running
  startMove("walk", 0.5);
  startMove("head", 0.5);
  nbRunsLeft = 0;
  autoSpeed = Eigen::Vector3d::Zero();

  // Ensuring fieldQThreshold is high enough
  if (RhIO::Root.getValueFloat("decision/fieldQThreshold").value < 0.9) {
    logger.warning("Low value for decision/fieldQThreshold");
  }
}

void LogMachine::step(float elapsed)
{
  time += elapsed;

  //Thread safe protection
  Move::bind->pull();

  //Retrieve service
  ModelService* service = Helpers::getServices()->model;

  // Getting robot status
  DecisionService * decision = getServices()->decision;

  bool incapacited = decision->isFallen || decision->handled;
  bool localized = time > localizeDuration && decision->isFieldQualityGood;

  if (state == STATE_INCAPACITED && !incapacited) {
    if (nbRunsLeft > 0) {
      logger.log("Robot is capable again -> going to preamble (%d runs left)", nbRunsLeft);
      setState(STATE_PREAMBLE);
    }
    else {
      logger.log("Robot is capable again -> going to inactive");
      setState(STATE_INACTIVE);
    }
  }
  else if (state != STATE_INCAPACITED && incapacited) {
    if (state != STATE_INACTIVE) {
      logger.log("Failed log: isFallen -> %d and handled -> %d",
                 decision->isFallen, decision->handled);
      endRun();
      if (state != STATE_PREAMBLE) {
        removeLastLog();
      }
    }
    setState(STATE_INCAPACITED);
  }
  else if (state == STATE_PREAMBLE && localized) {
    initialPosition =  getPosition();
    startRun();
    setState(STATE_LOGGING);
  }
  else if (state == STATE_EPILOGUE && localized) {
    // Getting final_pos in the src basis
    Eigen::Vector3d finalPosition = getPosition();
    double dxInWorld = finalPosition(0) - initialPosition(0);
    double dyInWorld = finalPosition(1) - initialPosition(1);
    double initTheta = initialPosition(2);
    double dx = cos(-initTheta) * dxInWorld - sin(-initTheta) * dyInWorld;
    double dy = sin(-initTheta) * dxInWorld + cos(-initTheta) * dyInWorld;
    double dtheta = normalizeRad(finalPosition(2) - initialPosition(2));
    Eigen::Vector3d diffInInitial(dx, dy, dtheta);
    logger.log("Ending log with delta: x: %lf [m], y: %lf [m], theta: %lf [deg]",
               dx, dy, rad2deg(dtheta));
    sequencesMutex.lock();
    sequences.back().targetDisplacements = diffInInitial;
    sequencesMutex.unlock();
    safetyBackup();
    nbRunsLeft = std::max(nbRunsLeft - 1, 0);
    // If there is other runs left to do go directly to logging mode (position is already known)
    if (nbRunsLeft > 0) {
      logger.log("Nb runs left: %d", nbRunsLeft);
      initialPosition =  getPosition();
      startRun();
      setState(STATE_LOGGING);
    }
    else {
      setState(STATE_INACTIVE);
    }
  }
  else if (state == STATE_LOGGING) {
    // Log current state if first step has been performed
    if (stepCount > 0) {
      sequencesMutex.lock();
      sequences.back().pushEntry(time,
                                 stepCount,
                                 service->readModel(), service->goalModel(),
                                 walk->getOrder(), walk->getPhase());
      sequencesMutex.unlock();
    }
    // Detect new steps in autoMode
    if (autoMode && walk->isNewStep()) {
      // Drive the walk at every step if we are not in fixed or custom modes
      if (!fixedSpeed && !customOrders) {
        Eigen::Matrix<double,3,2> deltaLimits = getDeltaLimits();
        Eigen::Matrix<double,3,2> stateLimits = getOrdersLimits();
        // Applying randomWalk
        randomWalk.uniformStepWithBounds(deltaLimits.col(0), deltaLimits.col(1),
                                         stateLimits.col(0), stateLimits.col(1),
                                         0);
        std::cout << "RandomWalk: " << randomWalk.statePos().transpose()
                  << std::endl;
        walk->setRawOrder(randomWalk.statePos(), true); 
      }
      stepCount++;
      // If max steps has been reached: stop current run
      if (stepCount > autoNbSteps) {
        endRun();
        if (autoMode) {
          setState(STATE_EPILOGUE);
        }
        else {
          safetyBackup();
          setState(STATE_INACTIVE);
        }
        std::cerr << "Automatically ending odometryCalibrationRun" << std::endl;
      }
    }
  }

  Move::bind->push();
}

void LogMachine::startRun()
{
  if (state == STATE_LOGGING) {
    throw std::logic_error("LogMachine::startRun: was already logging");
  }
  randomWalk.statePos().setZero();
  randomWalk.stateVel().setZero();
  stepCount = 0;
  if (customOrders) {
    autoSpeed = Eigen::Vector3d(cmdStep, cmdLat, cmdRot);
    logger.log("Starting log at custom speed: %f[m/step], %f[m/step], %f[rad/step]",
               autoSpeed(0), autoSpeed(1), autoSpeed(2));
  }
  else if (fixedSpeed) {
    autoSpeed = getRandomSpeed();
    logger.log("Starting log at fixed speed: %f[m/step], %f[m/step], %f[rad/step]",
               autoSpeed(0), autoSpeed(1), autoSpeed(2));
  }
  else {
    autoSpeed = Eigen::Vector3d(0,0,0);
  }
  walk->setRawOrder(autoSpeed,true);
  // Open a new sequence
  sequencesMutex.lock();
  sequences.push_back(Leph::OdometrySequence());
  sequencesMutex.unlock();
}

void LogMachine::endRun()
{
  if (state == STATE_INACTIVE) {
    throw std::logic_error("LogMachine::endRun: Not logging yet, can't end.");
  }
  // Stopping Motion
  walk->setRawOrder(0.0, 0.0, 0.0, false);
}

void LogMachine::removeLastLog()
{
  sequencesMutex.lock();
  if (sequences.size() == 0) {
    throw std::logic_error("LogMachine::removeLastLog: no log to remove");
  }
  sequences.pop_back();
  sequencesMutex.unlock();
}

void LogMachine::safetyBackup()
{
  sequencesMutex.lock();
  Leph::dumpOdometryDataToFile(sequences, ".logMachine.data");
  sequencesMutex.unlock();
}

std::string LogMachine::cmdSetLastRunPos(double dx, double dy, double da)
{
  sequencesMutex.lock();
  if (sequences.size() == 0) {
    sequencesMutex.unlock();
    throw std::logic_error("LogMachine::cmdSetLastRunPos: no active log");
  }
  sequences.back().targetDisplacements = Eigen::Vector3d(dx,dy,da);
  sequencesMutex.unlock();
  safetyBackup();

  return "Set target reference relative displacement to:"
    + std::string(" dX=") + std::to_string(dx)
    + std::string(" dY=") + std::to_string(dy)
    + std::string(" dA=") + std::to_string(da);
}

std::string LogMachine::cmdStartLog()
{
  if (autoMode) {
    setState(STATE_PREAMBLE);
  }
  else {
    startRun();
    setState(STATE_LOGGING);
  }
  return "Starting new learning sequence: Did you think about GyroTare?";
}

std::string LogMachine::cmdStartSequence(int nbRuns)
{
  if (!autoMode) {
    throw std::logic_error("StartSequence is only allowed in autoMode mode");
  }
  if (state != STATE_INACTIVE) {
    throw std::logic_error("StartSequence can only be callded from inactive state");
  }
  nbRunsLeft = nbRuns;
  setState(STATE_PREAMBLE);
  return "Started an auto log sequence of " + std::to_string(nbRuns) + " runs!"
    + " DID YOU THINK ABOUT GYROTARE ???";
}

std::string LogMachine::cmdStopLog()
{
  endRun();
  sequencesMutex.lock();
  std::ostringstream oss;
  oss << "Learning sequence stopped: "
      << stepCount << " steps in " << time << " ms ("
      << sequences.back().walkTrajsOrder.size() << " rows)";
  nbRunsLeft = 0;
  sequencesMutex.unlock();
  setState(STATE_INACTIVE);
  return oss.str();
}

std::string LogMachine::cmdCancelLog()
{
  if (state != STATE_INACTIVE) {
    throw std::logic_error("LogMachine::cmdCancelLog: currently logging");
  }

  sequencesMutex.lock();
  if (sequences.size() == 0) {
    sequencesMutex.unlock();
    throw std::logic_error("LogMachine::cmdCancelLog: no logs found");
  }
  sequences.pop_back();
  sequencesMutex.unlock();
  
  return "Learning sequence canceled";
}

std::string LogMachine::cmdClear()
{
  if (state != STATE_INACTIVE) {
    throw std::logic_error("LogMachine::cmdClear: state is not inactive");
  }

  sequencesMutex.lock();
  sequences.clear();
  sequencesMutex.unlock();

  return "All sequences have been cleared from memory";
}

std::string LogMachine::cmdDumpData(std::string filename)
{
  sequencesMutex.lock();
  Leph::dumpOdometryDataToFile(sequences, filename);
  sequencesMutex.unlock();
  return std::to_string(sequences.size()) + " have been dumped";
}

void LogMachine::enterState(std::string state)
{
  LocalisationService * loc = getServices()->localisation;
  time = 0;
  if (state == STATE_PREAMBLE) {
    loc->resetFieldFilter();
    head->setForceLocalize(true);
  }
  else if (state == STATE_LOGGING) {
    head->setDisabled(true);
  }
  else if (state == STATE_EPILOGUE) {
    loc->resetFieldFilter();
    head->setForceLocalize(true);
  }
  else if (state == STATE_INACTIVE) {
    head->setDisabled(true);
  }
  else if (state == STATE_INCAPACITED) {
    head->setDisabled(true);
  }
}

void LogMachine::exitState(std::string state)
{
  if (state == STATE_PREAMBLE) {
    head->setForceLocalize(false);
  }
  else if (state == STATE_LOGGING) {
    head->setDisabled(false);
  }
  else if (state == STATE_EPILOGUE) {
    head->setForceLocalize(false);
  }
  else if (state == STATE_INACTIVE) {
    head->setDisabled(false);
  }
  else if (state == STATE_INCAPACITED) {
    head->setDisabled(false);
  }
}

Eigen::Vector3d LogMachine::getPosition()
{
  LocalisationService * loc = getServices()->localisation;
  Point p = loc->getFieldPos();
  double theta = loc->getFieldOrientation();
  return Eigen::Vector3d(p.x, p.y, theta);
}

Eigen::Matrix<double,3,2> LogMachine::getOrdersLimits() {
  Eigen::Matrix<double,3,2> limits;
  limits.col(0) = walk->getMinOrders();
  limits.col(1) = walk->getMaxOrders();
  // Bounding states and explorations for randomWalk
  if (selectedDim >= 0) {
    double epsilon = std::pow(10,-10);
    for (int dim = 0; dim < 3; dim++) {
      if (dim != selectedDim) {
        limits(dim,0) = -epsilon;
        limits(dim,1) =  epsilon;
      }
      else if (selectedSign > 0) {
        limits(dim,0) = 0;
      }
      else if (selectedSign < 0) {
        limits(dim,1) = 0;
      }
    }
  }
  return limits;
}

Eigen::Matrix<double,3,2> LogMachine::getDeltaLimits() {
  Eigen::Matrix<double,3,2> limits;
  limits.col(0) = walk->getMinDeltaOrders();
  limits.col(1) = walk->getMaxDeltaOrders();
  // Bounding states and explorations for randomWalk
  if (selectedDim >= 0) {
    double epsilon = std::pow(10,-10);
    for (int dim = 0; dim < 3; dim++) {
      if (dim != selectedDim) {
        limits(dim,0) = -epsilon;
        limits(dim,1) =  epsilon;
      }
    }
  }
  return limits;
}

Eigen::Vector3d LogMachine::getRandomSpeed() {
  Eigen::Matrix<double,3,2> limits = getOrdersLimits();
  return rhoban_random::getUniformSamples(limits, 1, &engine)[0];
}
