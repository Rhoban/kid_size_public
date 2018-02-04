#include <stdexcept>
#include <RhIO.hpp>
#include <services/ModelService.h>
#include <Utils/AxisAngle.h>
#include <Utils/FileModelParameters.h>
#include <Model/JointModel.hpp>
#include <Model/NamesModel.h>
#include "TrajectoriesPlayer.hpp"

TrajectoriesPlayer::TrajectoriesPlayer() :
    _model(Leph::SigmabanModel),
    _filename(""),
    _modelParamsPath(""),
    _trajs(),
    _isFeedForward(false),
    _isLogging(false),
    _logPath(""),
    _t(-1.0),
    _timeGain(1.0),
    _isContinue(false),
    _offsetTrunkPos(),
    _offsetTrunkAxis(),
    _offsetFootPos(),
    _offsetFootAxis()
{
    //Initialize RhIO binding
    Move::initializeBinding();
    //Binding commands
    bind->bindFunc("trajectories_reset", "Reset TrajectoriesPlayer",
        &TrajectoriesPlayer::cmdReset, *this);
    bind->bindFunc("trajectories_start", "Start TrajectoriesPlayer",
        &TrajectoriesPlayer::cmdStart, *this);
    //Bind members
    bind->bindNew("filename", _filename, RhIO::Bind::PullOnly)
        ->defaultValue("/home/rhoban/trajKick.splines");
    bind->bindNew("modelParams", _modelParamsPath, RhIO::Bind::PullOnly)
        ->defaultValue("");
    bind->bindNew("t", _t, RhIO::Bind::PushOnly)
        ->defaultValue(-1.0);
    bind->bindNew("timeGain", _timeGain, RhIO::Bind::PullOnly)
        ->defaultValue(1.0);
    bind->bindNew("continue", _isContinue, RhIO::Bind::PullOnly)
        ->defaultValue(false);
    bind->bindNew("feedforward", _isFeedForward, RhIO::Bind::PullOnly)
        ->defaultValue(false);
    bind->bindNew("logPath", _logPath, RhIO::Bind::PullOnly)
        ->defaultValue("");
    //Offsets
    bind->bindNew("offsetTrunkPosX", _offsetTrunkPos.x(), 
        RhIO::Bind::PullOnly)->defaultValue(0.0);
    bind->bindNew("offsetTrunkPosY", _offsetTrunkPos.y(), 
        RhIO::Bind::PullOnly)->defaultValue(0.0);
    bind->bindNew("offsetTrunkPosZ", _offsetTrunkPos.z(), 
        RhIO::Bind::PullOnly)->defaultValue(0.0);
    bind->bindNew("offsetTrunkAxisX", _offsetTrunkAxis.x(), 
        RhIO::Bind::PullOnly)->defaultValue(0.0);
    bind->bindNew("offsetTrunkAxisY", _offsetTrunkAxis.y(), 
        RhIO::Bind::PullOnly)->defaultValue(0.0);
    bind->bindNew("offsetTrunkAxisZ", _offsetTrunkAxis.z(), 
        RhIO::Bind::PullOnly)->defaultValue(0.0);
    bind->bindNew("offsetFootPosX", _offsetFootPos.x(), 
        RhIO::Bind::PullOnly)->defaultValue(0.0);
    bind->bindNew("offsetFootPosY", _offsetFootPos.y(), 
        RhIO::Bind::PullOnly)->defaultValue(0.0);
    bind->bindNew("offsetFootPosZ", _offsetFootPos.z(), 
        RhIO::Bind::PullOnly)->defaultValue(0.0);
    bind->bindNew("offsetFootAxisX", _offsetFootAxis.x(), 
        RhIO::Bind::PullOnly)->defaultValue(0.0);
    bind->bindNew("offsetFootAxisY", _offsetFootAxis.y(), 
        RhIO::Bind::PullOnly)->defaultValue(0.0);
    bind->bindNew("offsetFootAxisZ", _offsetFootAxis.z(), 
        RhIO::Bind::PullOnly)->defaultValue(0.0);
}

std::string TrajectoriesPlayer::getName()
{
    return "TrajectoriesPlayer";
}

void TrajectoriesPlayer::onStart()
{
    _t = -1.0;
    _isLogging = false;
}

void TrajectoriesPlayer::step(float elapsed)
{
    //Import RhIO binded variables
    bind->pull();
    
    //Retrieve model service
    ModelService* service = 
        Helpers::getServices()->model;

    //Check if initialized
    if (_filename == "" || _trajs.size() == 0) {
        //Export RhIO binded variables
        bind->push();
        //No update without data
        return;
    }

    //Get trajectory phase time
    double t = 0.0;
    if (_t < 0.0) {
        t = 0.0;
    } else if (_t < _trajs.max()) {
        t = _t;
        _t += _timeGain * elapsed;
        if (_isContinue && _t >= _trajs.max()) {
            _t -= _trajs.max();
        } 
    } else {
        t = _trajs.max() - 0.001;
        //Stop logging if ended
        if (_isLogging) {
            service->stopLogging();
            _isLogging = false;
        }
    }
        
    //Implement Feed Forward correction
    bool isSuccess = false;
    if (_isFeedForward) {
        //Joint model with default parameters
        Leph::JointModel joint;
        //Compute kinematics position, velocity and
        //acceleration on goal trajectories
        Eigen::VectorXd dq;
        Eigen::VectorXd ddq;
        isSuccess = Leph::TrajectoriesComputeKinematics(
            t, _trajs, _model, dq, ddq);

        //Retrieve double support state
        bool isDoubleSupport;
        Leph::HumanoidFixedModel::SupportFoot supportFoot;
        Leph::TrajectoriesSupportFootState(t, _trajs,
            isDoubleSupport, supportFoot);
        //Compute external torque on goal trajectories
        Eigen::VectorXd torques;
        if (isDoubleSupport) {
            if (supportFoot == Leph::HumanoidFixedModel::LeftSupportFoot) {
                torques = _model.get().inverseDynamicsClosedLoop(
                    "right_foot_tip", nullptr, false, dq, ddq);
            } else {
                torques = _model.get().inverseDynamicsClosedLoop(
                    "left_foot_tip", nullptr, false, dq, ddq);
            }
        } else {
            torques = _model
                .get().inverseDynamics(dq, ddq);
        }
        
        for (const std::string& name : Leph::NamesDOFLeg) {
            //Log DOF target position (real goal)
            service->logValue("target:"+name, 
                _model.get().getDOF(name));
            //Compute feed forward offset
            size_t index = _model.get().getDOFIndex(name);
            double offset = joint.computeFeedForward(
                dq(index), ddq(index), torques(index));
            _model.get().setDOF(
                name, offset + _model.get().getDOF(name));
        }
    } else {
        //Compute cartesian target
        Eigen::Vector3d trunkPos;
        Eigen::Vector3d trunkAxis;
        Eigen::Vector3d footPos;
        Eigen::Vector3d footAxis;
        bool isDoubleSupport;
        Leph::HumanoidFixedModel::SupportFoot supportFoot;
        Leph::TrajectoriesTrunkFootPos(
            t, _trajs, 
            trunkPos, trunkAxis,
            footPos, footAxis);
        Leph::TrajectoriesSupportFootState(
            t, _trajs,
            isDoubleSupport, 
            supportFoot);

        //Apply user offsets
        trunkPos += _offsetTrunkPos;
        trunkAxis += _offsetTrunkAxis;
        footPos += _offsetFootPos;
        footAxis += _offsetFootAxis;

        //Compute DOF positions
        isSuccess = _model.trunkFootIK(
            supportFoot,
            trunkPos,
            Leph::AxisToMatrix(trunkAxis),
            footPos,
            Leph::AxisToMatrix(footAxis));
        
        for (const std::string& name : Leph::NamesDOFLeg) {
            //Log DOF target position (real goal)
            service->logValue("target:"+name, 
                _model.get().getDOF(name));
        }
    }
    
    //Check IK result
    if (!isSuccess) {
        _t = -1.0;
        if (_isLogging) {
            service->stopLogging();
            _isLogging = false;
        }
        return;
    }

    //Retrieve model
    Leph::HumanoidFixedModel& goalModel = 
        Helpers::getServices()->model->goalModel();
    //Copy DOF target to goalModel
    for (const std::string& name : Leph::NamesDOF) {
        goalModel.get().setDOF(name, _model.get().getDOF(name));
    }
    
    //Flush to lowlevel
    service->flushLegs();
    
    //Export RhIO binded variables
    bind->push();
}

void TrajectoriesPlayer::cmdReset()
{
    _t = -1.0;
    _isLogging = false;
    _trajs = Leph::Trajectories();
    _trajs.importData(_filename);
    //Load model parameters
    Eigen::MatrixXd jointData;
    std::map<std::string, size_t> jointName;
    Eigen::MatrixXd inertiaData;
    std::map<std::string, size_t> inertiaName;
    Eigen::MatrixXd geometryData;
    std::map<std::string, size_t> geometryName;
    if (_modelParamsPath != "") {
        Leph::ReadModelParameters(
            _modelParamsPath,
            jointData, jointName,
            inertiaData, inertiaName,
            geometryData, geometryName);
    }
    //Reset model
    _model = Leph::HumanoidFixedModel(
        Leph::SigmabanModel,
        inertiaData,
        inertiaName,
        geometryData,
        geometryName);
}

std::string TrajectoriesPlayer::cmdStart()
{
    //Retrieve model service
    ModelService* service = 
        Helpers::getServices()->model;
    //Start logging if enabled
    if (_logPath != "") {
        service->startLogging(_logPath, false);
        _isLogging = true;
    }
    //Set time to begin
    _t = 0.0;
    return "Start";
}

