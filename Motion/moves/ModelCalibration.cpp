#include <stdexcept>
#include <RhIO.hpp>
#include <Utils/AxisAngle.h>
#include <Utils/time.h>
#include "services/ModelService.h"
#include "ModelCalibration.hpp"

/**
 * Stable left single support
 * static position
 */
static Eigen::Vector3d staticTrunkPos(
    -0.00557785331559037, -0.0115849568418458, 0.285);
static Eigen::Vector3d staticTrunkAxis(
    -0.672036398746933, 0.0743358280850477, 0.0028323027017884);
static Eigen::Vector3d staticFootPos(
    0.0208647084129351, -0.095, 0.0591693358237435);
static Eigen::Vector3d staticFootAxis(
    0.0, 0.0, 0.0);
static Leph::HumanoidFixedModel::SupportFoot staticSupport(
    Leph::HumanoidFixedModel::LeftSupportFoot);

ModelCalibration::ModelCalibration() :
    Move(),
    _mutex(),
    _countWait(0),
    _logPrefix("/tmp/calibration_log_"),
    _motionName(""),
    _motionTime(-2.0),
    _errorMsg(""),
    _motionIsSmooth(),
    _motionPeriod(),
    _motionAngleAmpl(),
    _motionCartAmpl(),
    _motionAngleStart(),
    _motionCartStart(),
    _motionTrajectory(""),
    _trajs()
{
    //Initialize RhIO binding
    Move::initializeBinding();

    //Binding
    bind->bindNew("savePrefix", _logPrefix, RhIO::Bind::PullOnly)
        ->comment("Log prefix path")
        ->defaultValue(_logPrefix);
    bind->bindNew("errorMessage", _errorMsg, RhIO::Bind::PushOnly)
        ->comment("Calibration error message")
        ->defaultValue(_logPrefix);
    bind->bindNew("motionIsSmooth", _motionIsSmooth, RhIO::Bind::PullOnly)
        ->comment("Motion use smooth or step spline")
        ->defaultValue(true);
    bind->bindNew("motionPeriod", _motionPeriod, RhIO::Bind::PullOnly)
        ->comment("Motion time lenght parameter in seconds")
        ->defaultValue(1.0);
    bind->bindNew("motionAngleAmpl", _motionAngleAmpl, RhIO::Bind::PullOnly)
        ->comment("Motion angular amplitude parameter in degrees")
        ->defaultValue(20.0);
    bind->bindNew("motionCartAmpl", _motionCartAmpl, RhIO::Bind::PullOnly)
        ->comment("Motion cartesian amplitude parameter in meters")
        ->defaultValue(0.01);
    bind->bindNew("motionAngleStart", _motionAngleStart, RhIO::Bind::PullOnly)
        ->comment("Motion angular start offset parameter in degrees")
        ->defaultValue(0.0);
    bind->bindNew("motionCartStart", _motionCartStart, RhIO::Bind::PullOnly)
        ->comment("Motion cartesian start offset parameter in meters")
        ->defaultValue(0.0);
    bind->bindNew("motionTrajectory", _motionTrajectory, RhIO::Bind::PullOnly)
        ->comment("Motion trajectories filename")
        ->defaultValue("");

    //Bind command
    bind->bindFunc("calibration_model_start", 
        "Start given calibration sequence",
        &ModelCalibration::cmdStart, *this);
}

std::string ModelCalibration::getName()
{
    return "ModelCalibration";
}

void ModelCalibration::onStart()
{
    //State reset
    _countWait = 0;
    _motionTime = -2.0;
    _motionName = "";
}

void ModelCalibration::step(float elapsed)
{
    _mutex.lock();
    //Bind pull
    bind->pull();

    //Retrieve model service
    ModelService* service = 
        Helpers::getServices()->model;
    //Retrieve goal model
    Leph::HumanoidFixedModel& goalModel = 
        service->goalModel();

    //Default static position
    bool isSuccess = goalModel.trunkFootIK(
        staticSupport,
        staticTrunkPos,
        Leph::AxisToMatrix(staticTrunkAxis),
        staticFootPos,
        Leph::AxisToMatrix(staticFootAxis));
    if (!isSuccess) {
        _errorMsg = "IK Error at time=" 
            + std::to_string(_motionTime);
    }
    goalModel.get().setDOF("left_shoulder_pitch", 0.0);
    goalModel.get().setDOF("left_shoulder_roll", 0.3);
    goalModel.get().setDOF("left_elbow", 0.0);
    goalModel.get().setDOF("right_shoulder_pitch", 0.0);
    goalModel.get().setDOF("right_shoulder_roll", -0.3);
    goalModel.get().setDOF("right_elbow", 0.0);
    goalModel.get().setDOF("head_yaw", 0.0);
    goalModel.get().setDOF("head_pitch", 0.0);

    if (_countWait > 0) {
        //Wait for log to be written
        //and motion initialization
        _countWait--;
    } else if (_motionName == "") {
        //Check if a calibration movement 
        //is currently beeing played
        _motionTime = -2.0;
    } else {
        //Actual used time
        double time = 0.0;
        //Motion initialization
        if (_motionTime > -1.5 && _motionTime < 0.0) {
            //First iteration case
            //Start logging
            service->startLogging(
                _logPrefix
                + _motionName 
                + std::string("_" )
                + Leph::currentDate()
                + std::string(".mapseries"), 
                false);
            //Update to playing state
            _motionTime = 0.0;
            time = 0.0;
        } else if (_motionTime < -1.5) {
            //Update to start log state
            _motionTime = -1.0;
            //Wait motion position initialization
            _countWait = 100;
            time = 0.0;
        } else {
            time = _motionTime;
        }
        //Motion finish time
        double endTime = 0.0;
        //Motion implementation
        if (_motionName == "shoulder_pitch") {
            Leph::Spline spline;
            if (_motionIsSmooth) {
                spline = buildSmoothTraj(
                    _motionAngleStart, _motionPeriod, 
                    _motionAngleAmpl*M_PI/180.0, false);
            } else {
                spline = buildStepTraj(
                    _motionAngleStart, _motionPeriod, 
                    _motionAngleAmpl*M_PI/180.0, false);
            }
            endTime = spline.max();
            goalModel.get().setDOF("left_shoulder_pitch", spline.pos(time));
        } else if (_motionName == "foot_x") {
            Leph::Spline spline;
            if (_motionIsSmooth) {
                spline = buildSmoothTraj(
                    _motionCartStart, _motionPeriod, 
                    _motionCartAmpl, false);
            } else {
                spline = buildStepTraj(
                    _motionCartStart, _motionPeriod, 
                    _motionCartAmpl, false);
            }
            endTime = spline.max();
            bool isSuccess = goalModel.trunkFootIK(
                staticSupport,
                staticTrunkPos,
                Leph::AxisToMatrix(staticTrunkAxis),
                staticFootPos + Eigen::Vector3d(spline.pos(time), 0.0, 0.0),
                Leph::AxisToMatrix(staticFootAxis));
            if (!isSuccess) {
                _errorMsg = "IK Error at time=" 
                    + std::to_string(_motionTime);
            }
        } else if (_motionName == "foot_y") {
            Leph::Spline spline;
            if (_motionIsSmooth) {
                spline = buildSmoothTraj(
                    _motionCartStart, _motionPeriod, 
                    _motionCartAmpl, false);
            } else {
                spline = buildStepTraj(
                    _motionCartStart, _motionPeriod, 
                    _motionCartAmpl, false);
            }
            endTime = spline.max();
            bool isSuccess = goalModel.trunkFootIK(
                staticSupport,
                staticTrunkPos,
                Leph::AxisToMatrix(staticTrunkAxis),
                staticFootPos + Eigen::Vector3d(0.0, spline.pos(time), 0.0),
                Leph::AxisToMatrix(staticFootAxis));
            if (!isSuccess) {
                _errorMsg = "IK Error at time=" 
                    + std::to_string(_motionTime);
            }
        } else if (_motionName == "foot_z") {
            Leph::Spline spline;
            if (_motionIsSmooth) {
                spline = buildSmoothTraj(
                    _motionCartStart, _motionPeriod, 
                    _motionCartAmpl, false);
            } else {
                spline = buildStepTraj(
                    _motionCartStart, _motionPeriod, 
                    _motionCartAmpl, false);
            }
            endTime = spline.max();
            bool isSuccess = goalModel.trunkFootIK(
                staticSupport,
                staticTrunkPos,
                Leph::AxisToMatrix(staticTrunkAxis),
                staticFootPos + Eigen::Vector3d(0.0, 0.0, spline.pos(time)),
                Leph::AxisToMatrix(staticFootAxis));
            if (!isSuccess) {
                _errorMsg = "IK Error at time=" 
                    + std::to_string(_motionTime);
            }
        } else if (_motionName == "trunk_x") {
            Leph::Spline spline;
            if (_motionIsSmooth) {
                spline = buildSmoothTraj(
                    _motionCartStart, _motionPeriod, 
                    _motionCartAmpl, false);
            } else {
                spline = buildStepTraj(
                    _motionCartStart, _motionPeriod, 
                    _motionCartAmpl, false);
            }
            endTime = spline.max();
            bool isSuccess = goalModel.trunkFootIK(
                staticSupport,
                staticTrunkPos + Eigen::Vector3d(spline.pos(time), 0.0, 0.0),
                Leph::AxisToMatrix(staticTrunkAxis),
                staticFootPos,
                Leph::AxisToMatrix(staticFootAxis));
            if (!isSuccess) {
                _errorMsg = "IK Error at time=" 
                    + std::to_string(_motionTime);
            }
        } else if (_motionName == "trunk_y") {
            Leph::Spline spline;
            if (_motionIsSmooth) {
                spline = buildSmoothTraj(
                    _motionCartStart, _motionPeriod, 
                    _motionCartAmpl, false);
            } else {
                spline = buildStepTraj(
                    _motionCartStart, _motionPeriod, 
                    _motionCartAmpl, false);
            }
            endTime = spline.max();
            bool isSuccess = goalModel.trunkFootIK(
                staticSupport,
                staticTrunkPos + Eigen::Vector3d(0.0, spline.pos(time), 0.0),
                Leph::AxisToMatrix(staticTrunkAxis),
                staticFootPos,
                Leph::AxisToMatrix(staticFootAxis));
            if (!isSuccess) {
                _errorMsg = "IK Error at time=" 
                    + std::to_string(_motionTime);
            }
        } else if (_motionName == "trunk_z") {
            Leph::Spline spline;
            if (_motionIsSmooth) {
                spline = buildSmoothTraj(
                    _motionCartStart, _motionPeriod, 
                    _motionCartAmpl, false);
            } else {
                spline = buildStepTraj(
                    _motionCartStart, _motionPeriod, 
                    _motionCartAmpl, false);
            }
            endTime = spline.max();
            bool isSuccess = goalModel.trunkFootIK(
                staticSupport,
                staticTrunkPos + Eigen::Vector3d(0.0, 0.0, spline.pos(time)),
                Leph::AxisToMatrix(staticTrunkAxis),
                staticFootPos,
                Leph::AxisToMatrix(staticFootAxis));
            if (!isSuccess) {
                _errorMsg = "IK Error at time=" 
                    + std::to_string(_motionTime);
            }
        } else if (_motionName == "trajectory") {
            //Load trajectories during initialization
            if (_motionTime < 0.0) {
                _trajs = Leph::Trajectories();
                _trajs.importData(_motionTrajectory);
            }
            //Compute cartesian target
            Eigen::Vector3d trunkPos;
            Eigen::Vector3d trunkAxis;
            Eigen::Vector3d footPos;
            Eigen::Vector3d footAxis;
            bool isDoubleSupport;
            Leph::HumanoidFixedModel::SupportFoot supportFoot;
            Leph::TrajectoriesTrunkFootPos(
                time, _trajs, 
                trunkPos, trunkAxis,
                footPos, footAxis);
            Leph::TrajectoriesSupportFootState(
                time, _trajs,
                isDoubleSupport, 
                supportFoot);
            endTime = _trajs.max();
            //Run inverse kinematics
            bool isSuccess = goalModel.trunkFootIK(
                supportFoot,
                trunkPos,
                Leph::AxisToMatrix(trunkAxis),
                footPos,
                Leph::AxisToMatrix(footAxis));
            if (!isSuccess) {
                _errorMsg = "IK Error at time=" 
                    + std::to_string(_motionTime);
            }
        } else {
            //Invalid motion name
            _errorMsg = "Unknown motion name=" + _motionName;
            _motionName = "";
            _motionTime = -2.0;
        }
        //Update motion state
        if (
            _motionName != "" && 
            _motionTime >= 0.0
        ) {
            //Increase playing time
            _motionTime += elapsed;
        }
        //Motion end
        if (
            _motionName != "" && 
            _motionTime >= 0.0 && 
            _motionTime >= endTime
        ) {
            //Stop log and wait
            service->stopLogging();
            _countWait = 100;
            //Reset
            _motionName = "";
            _motionTime = -2.0;
        }
    }
    
    //Send targets to low level
    service->flushAll();
    
    //Bind push
    bind->push();
    _mutex.unlock();
}

Leph::Spline ModelCalibration::buildStepTraj(
    double start, double period, 
    double ampl, bool isHalf)
{
    //Time delay before and after moving
    double pauseTime = 1.0;

    Leph::SmoothSpline spline;
    double time = 0.0;
    spline.addPoint(time, start, 0.0, 0.0);
    time += pauseTime;
    spline.addPoint(time, start, 0.0, 0.0);
    if (isHalf) {
        time += 0.0;
        spline.addPoint(time, ampl, 0.0, 0.0);
        time += pauseTime;
        spline.addPoint(time, ampl, 0.0, 0.0);
    } else {
        spline.addPoint(time, ampl, 0.0, 0.0);
        time += period;
        spline.addPoint(time, ampl, 0.0, 0.0);
        spline.addPoint(time, -ampl, 0.0, 0.0);
        time += period;
        spline.addPoint(time, -ampl, 0.0, 0.0);
        spline.addPoint(time, start, 0.0, 0.0);
        time += pauseTime;
        spline.addPoint(time, start, 0.0, 0.0);
    }
    return spline;
}
Leph::Spline ModelCalibration::buildSmoothTraj(
    double start, double period, 
    double ampl, bool isHalf)
{
    //Time delay before and after moving
    double pauseTime = 1.0;
    
    Leph::SmoothSpline spline;
    double time = 0.0;
    spline.addPoint(time, start, 0.0, 0.0);
    time += pauseTime;
    spline.addPoint(time, start, 0.0, 0.0);
    if (isHalf) {
        time += period;
        spline.addPoint(time, ampl, 0.0, 0.0);
        time += pauseTime;
        spline.addPoint(time, ampl, 0.0, 0.0);
    } else {
        time += period;
        spline.addPoint(time, ampl, 0.0, 0.0);
        time += period;
        spline.addPoint(time, -ampl, 0.0, 0.0);
        time += period;
        spline.addPoint(time, start, 0.0, 0.0);
        time += pauseTime;
        spline.addPoint(time, start, 0.0, 0.0);
    }
    return spline;
}
        
std::string ModelCalibration::cmdStart(std::string name)
{
    _mutex.lock();
    _motionName = name;
    _motionTime = -2.0;
    _mutex.unlock();
    return 
        "Start calibration sequence: " + name 
        + " with prefix: " 
        + _logPrefix;
}

