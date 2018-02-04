#include <fstream>
#include "moves/OdometryCalibration.hpp"
#include "services/ModelService.h"
#include "Walk.h"

OdometryCalibration::OdometryCalibration(Walk* walk) :
    _walk(walk),
    _message(""),
    _displacement(0.0, 0.0, -1.0),
    _isLogging(false),
    _readTrajsPose(),
    _readTrajsSupport(),
    _goalTrajsPose(),
    _goalTrajsSupport(),
    _walkTrajsOrder(),
    _walkTrajsPhase(),
    _targetDisplacements(),
    _mutex(),
    _dumpFileName(""),
    _isAutoWalk(false),
    _autoEnable(false),
    _autoMinStep(),
    _autoMaxStep(),
    _autoMinLateral(),
    _autoMaxLateral(),
    _autoMinTurn(),
    _autoMaxTurn(),
    _autoDeltaStep(),
    _autoDeltaLateral(),
    _autoDeltaTurn(),
    _autoInertia(0.0),
    _autoLastPhase(-1.0),
    _randomWalk(3)
{
    Move::initializeBinding();
    //RhIO Binding function
    bind->bindFunc("calibration_odometry_reference", 
        "Set target reference displacement <dX> <dY> <dA|-1> <setLast|false> in source frame." +
        std::string("Assign last sequence with displacement if setLast is true"),
        &OdometryCalibration::cmdReferenceDisplacement, *this, {"", "", "-1.0", "false"});
    bind->bindFunc("calibration_odometry_start", 
        "Start logging a learning sequence",
        &OdometryCalibration::cmdStartLog, *this);
    bind->bindFunc("calibration_odometry_stop", 
        "Stop logging a learning sequence",
        &OdometryCalibration::cmdStopLog, *this);
    bind->bindFunc("calibration_odometry_cancel", 
        "Stop logging and pop last started learning sequence.",
        &OdometryCalibration::cmdCancelLog, *this);
    bind->bindFunc("calibration_odometry_dump", 
        "Dump recorded odometry data into given filename",
        &OdometryCalibration::cmdDumpData, *this);
    
    bind->bindNew("autoEnable", _isAutoWalk, RhIO::Bind::PullOnly)
        ->defaultValue(false)->persisted(true)
        ->comment("Enable random walk exploration");
    bind->bindNew("autoInertia", _autoInertia, RhIO::Bind::PullOnly)
        ->defaultValue(0.0)->persisted(true);
    bind->bindNew("autoStepMin", _autoMinStep, RhIO::Bind::PullOnly)
        ->defaultValue(-0.02)->persisted(true);
    bind->bindNew("autoStepMax", _autoMaxStep, RhIO::Bind::PullOnly)
        ->defaultValue(0.04)->persisted(true);
    bind->bindNew("autoLateralMin", _autoMinLateral, RhIO::Bind::PullOnly)
        ->defaultValue(-0.02)->persisted(true);
    bind->bindNew("autoLateralMax", _autoMaxLateral, RhIO::Bind::PullOnly)
      ->defaultValue(0.02)->persisted(true);
    bind->bindNew("autoTurnMin", _autoMinTurn, RhIO::Bind::PullOnly)
      ->defaultValue(-0.3)->persisted(true);
    bind->bindNew("autoTurnMax", _autoMaxTurn, RhIO::Bind::PullOnly)
      ->defaultValue(0.3)->persisted(true);
    bind->bindNew("autoStepDelta", _autoDeltaStep, RhIO::Bind::PullOnly)
      ->defaultValue(0.01)->persisted(true);
    bind->bindNew("autoLateralDelta", _autoDeltaLateral, RhIO::Bind::PullOnly)
      ->defaultValue(0.01)->persisted(true);
    bind->bindNew("autoTurnDelta", _autoDeltaTurn, RhIO::Bind::PullOnly)
      ->defaultValue(0.1)->persisted(true);
    bind->bindNew("autoNbCycles", _autoNbCycles, RhIO::Bind::PullOnly)
      ->defaultValue(40)->persisted(true)
      ->comment("Number of cycles when doing random exploration");
    bind->bindNew("cycleCount", _cycleCounts, RhIO::Bind::PushOnly)
      ->defaultValue(0)
      ->comment("Number of cycles since beginning of random exploration");
}

std::string OdometryCalibration::getName()
{
    return "OdometryCalibration";
}
void OdometryCalibration::onStart()
{
    _readTrajsPose.clear();
    _readTrajsSupport.clear();
    _goalTrajsPose.clear();
    _goalTrajsSupport.clear();
    _walkTrajsOrder.clear();
    _walkTrajsPhase.clear();
    _targetDisplacements.clear();
    _displacement = Eigen::Vector3d(0.0, 0.0, -1.0);
    _isLogging = false;
    _dumpFileName = "";
    _autoLastPhase = -1.0;
    _randomWalk.statePos().setZero();
    _randomWalk.stateVel().setZero();
    _autoEnable = false;
    _autoNbCycles = 0;
}
void OdometryCalibration::step(float elapsed)
{
    //Elapsed is not used in this move
    (void)elapsed;

    //Thread safe protection
    std::lock_guard<std::mutex> lock(_mutex);
    Move::bind->pull();

    //Retrieve service
    ModelService* service = Helpers::getServices()->model;

    if (_isLogging) {
        //Retrieve Models
        Leph::HumanoidFixedModel& readModel = service->readModel();
        Leph::HumanoidFixedModel& goalModel = service->goalModel();
        //Save models state dans support
        _readTrajsPose.back().push_back(readModel.get().getPose());
        _readTrajsSupport.back().push_back(readModel.getSupportFoot());
        _goalTrajsPose.back().push_back(goalModel.get().getPose());
        _goalTrajsSupport.back().push_back(goalModel.getSupportFoot());
        _walkTrajsOrder.back().push_back(_walk->getRawOrder());
        _walkTrajsPhase.back().push_back(_walk->getPhase());
    }

    if (_dumpFileName != "") {
        std::ofstream file(_dumpFileName);
        for (size_t i=0;i<this->_readTrajsPose.size();i++) {
            for (size_t j=0;j<this->_readTrajsPose[i].size();j++) {
                file 
                    << i << " "
                    << j << " "
                    << _readTrajsPose[i][j].x() << " "
                    << _readTrajsPose[i][j].y() << " "
                    << _readTrajsPose[i][j].z() << " "
                    << (int)_readTrajsSupport[i][j] << " "
                    << _goalTrajsPose[i][j].x() << " "
                    << _goalTrajsPose[i][j].y() << " "
                    << _goalTrajsPose[i][j].z() << " "
                    << (int)_goalTrajsSupport[i][j] << " "
                    << _walkTrajsOrder[i][j].x() << " "
                    << _walkTrajsOrder[i][j].y() << " "
                    << _walkTrajsOrder[i][j].z() << " "
                    << _walkTrajsOrder[i][j](3) << " "
                    << _walkTrajsPhase[i][j] << " "
                    << _targetDisplacements[i].x() << " "
                    << _targetDisplacements[i].y() << " "
                    << _targetDisplacements[i].z()
                    << std::endl;
            }
        }
        file.close();
        _dumpFileName = "";
    }

    //Drive the walk using random process
    if (
        _autoEnable && 
        _autoLastPhase > 0.8 && 
        _walk->getPhase() < 0.2
    ) {
        Eigen::VectorXd deltaLow(3);
        Eigen::VectorXd deltaUp(3);
        Eigen::VectorXd stateLow(3);
        Eigen::VectorXd stateUp(3);
        deltaLow << -_autoDeltaStep, -_autoDeltaLateral, -_autoDeltaTurn;
        deltaUp  << _autoDeltaStep, _autoDeltaLateral, _autoDeltaTurn;
        stateLow << _autoMinStep, _autoMinLateral, _autoMinTurn;
        stateUp  << _autoMaxStep, _autoMaxLateral, _autoMaxTurn;
        _randomWalk.uniformStepWithBounds(
            deltaLow, deltaUp, 
            stateLow, stateUp, 
            _autoInertia);
        _cycleCounts++;
        // If number of cycles is finished, the 
        if (_cycleCounts <= _autoNbCycles) {
            _walk->setRawOrder(
                _randomWalk.statePos().x(), 
                _randomWalk.statePos().y(), 
                _randomWalk.statePos().z(), 
                true);
        }
        else {
            _isLogging = false;
            _autoEnable = false;
            _walk->setRawOrder(0.0, 0.0, 0.0, false);
            std::cerr << "Automatically ending odometryCalibrationRun" << std::endl;
        }
    }
    _autoLastPhase = _walk->getPhase();
    
    Move::bind->push();
}

std::string OdometryCalibration::cmdReferenceDisplacement(
    double dx, double dy, double da, bool setLast)
{
    std::lock_guard<std::mutex> lock(_mutex);
    _displacement.x() = dx;
    _displacement.y() = dy;
    _displacement.z() = da;
    if (setLast && _targetDisplacements.size() > 0) {
        _targetDisplacements.back() = _displacement;
    }
    return "Set target reference relative displacement to:"
        + std::string(" dX=") + std::to_string(dx)
        + std::string(" dY=") + std::to_string(dy)
        + std::string(" dA=") + std::to_string(da);
}

std::string OdometryCalibration::cmdStartLog()
{
    std::lock_guard<std::mutex> lock(_mutex);
    _isLogging = true;
    _readTrajsPose.push_back(
        std::vector<Eigen::Vector3d>());
    _readTrajsSupport.push_back(
        std::vector<Leph::HumanoidFixedModel::SupportFoot>());
    _goalTrajsPose.push_back(
        std::vector<Eigen::Vector3d>());
    _goalTrajsSupport.push_back(
        std::vector<Leph::HumanoidFixedModel::SupportFoot>());
    _walkTrajsOrder.push_back(
        std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>>());
    _walkTrajsPhase.push_back(
        std::vector<double>());
    _targetDisplacements.push_back(_displacement);
    if (_isAutoWalk) {
        _autoEnable = true;
        RhIO::Root.setBool("/moves/walk/noSmoothingMode", true);
        _cycleCounts = 0;
        _randomWalk.statePos().setZero();
        _randomWalk.stateVel().setZero();
    }

    return "Starting new learning sequence with target displacement: "
        + std::string(" dX=") + std::to_string(_displacement.x())
        + std::string(" dY=") + std::to_string(_displacement.y())
        + std::string(" dA=") + std::to_string(_displacement.z());
}
std::string OdometryCalibration::cmdStopLog()
{
    std::lock_guard<std::mutex> lock(_mutex);
    _isLogging = false;
    _autoEnable = false;
    if (_isAutoWalk) {
        _walk->setRawOrder(0.0, 0.0, 0.0, false);
    }
    return "Stopping learning sequence " 
        + std::to_string(_readTrajsPose.size())
        + std::string(" length=")
        + std::to_string(_readTrajsPose.back().size());
}
std::string OdometryCalibration::cmdCancelLog()
{
    std::lock_guard<std::mutex> lock(_mutex);
    _isLogging = false;
    if (_readTrajsPose.size() > 0) {
        _readTrajsPose.pop_back();
        _readTrajsSupport.pop_back();
        _goalTrajsPose.pop_back();
        _goalTrajsSupport.pop_back();
        _walkTrajsOrder.pop_back();
        _walkTrajsPhase.pop_back();
        _targetDisplacements.pop_back();
    }
    _autoEnable = false;
    if (_isAutoWalk) {
        _walk->setRawOrder(0.0, 0.0, 0.0, false);
    }
    return "Cancelling learning sequence " 
        + std::to_string(_readTrajsPose.size())
        + std::string(" length=")
        + std::to_string(_readTrajsPose.back().size());
}

std::string OdometryCalibration::cmdDumpData(std::string filename)
{
    std::lock_guard<std::mutex> lock(_mutex);
    _dumpFileName = filename;
    return "Dump odometry recorded data into: " + filename;
}

