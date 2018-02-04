#include <stdexcept>
#include <RhIO.hpp>
#include <services/ModelService.h>
#include <Utils/AxisAngle.h>
#include "KickPhilipp.hpp"

KickPhilipp::KickPhilipp() :
    _filename(),
    _trajs(),
    _t(-1.0),
    _timeGain(1.0)
{
    //Initialize RhIO binding
    Move::initializeBinding();
    //Binding commands
    bind->bindFunc("kick_test_reset", "Reset KickPhilipp",
        &KickPhilipp::cmdReset, *this);
    bind->bindFunc("kick_test_start", "Start KickPhilipp",
        &KickPhilipp::cmdStart, *this);
    //Bind members
    bind->bindNew("filename", _filename, RhIO::Bind::PullOnly)
        ->defaultValue("/home/rhoban/trajKick_vel1.0_x0.02_2016-08-18-12-46-56.splines");
    bind->bindNew("t", _t, RhIO::Bind::PushOnly)
        ->defaultValue(-1.0);
    bind->bindNew("timeGain", _timeGain, RhIO::Bind::PullOnly)
        ->defaultValue(1.0);
}

std::string KickPhilipp::getName()
{
    return "KickPhilipp";
}

void KickPhilipp::onStart()
{
    _t = -1.0;
}

void KickPhilipp::step(float elapsed)
{
    //Import RhIO binded variables
    bind->pull();

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
    } else {
        t = _trajs.max() - 0.001;
    }
    
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

    //Retrieve model
    Leph::HumanoidFixedModel& goalModel = 
        Helpers::getServices()->model->goalModel();

    //Compute DOF positions
    bool isSuccess = goalModel.trunkFootIK(
        supportFoot,
        trunkPos,
        Leph::AxisToMatrix(trunkAxis),
        footPos,
        Leph::AxisToMatrix(footAxis));
    if (!isSuccess) {
        _t = -1.0;
        return;
    }

    //Flush to lowlevel
    Helpers::getServices()->model->flushLegs();
    
    //Export RhIO binded variables
    bind->push();
}

void KickPhilipp::cmdReset()
{
    _t = -1.0;
    _trajs = Leph::Trajectories();
    _trajs.importData(_filename);
}

std::string KickPhilipp::cmdStart()
{
    _t = 0.0;
    return "Start";
}

