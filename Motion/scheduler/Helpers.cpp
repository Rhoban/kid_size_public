#include <stdexcept>
#include <rhoban_utils/timing/time_stamp.h>
#include <rhoban_utils/logging/logger.h>
#include "services/Services.h"
#include "scheduler/MoveScheduler.h"
#include "services/ModelService.h"
#include "Helpers.h"
#include <Devices/GY85.hpp>


using namespace rhoban_utils;

static rhoban_utils::Logger out("helpers");

Helpers::Helpers() :
        _scheduler(nullptr)
{
}

void Helpers::setScheduler(MoveScheduler* scheduler)
{
    _scheduler = scheduler;
    if (_scheduler == nullptr) {
        throw std::logic_error("Helpers null pointer");
    }
}

const MoveScheduler* Helpers::getScheduler() const
{
    return _scheduler;
}

MoveScheduler* Helpers::getScheduler()
{
    return _scheduler;
}

Services* Helpers::getServices()
{
    return _scheduler->getServices();
}
const Services* Helpers::getServices() const
{
    return _scheduler->getServices();
}
Moves* Helpers::getMoves()
{
    return _scheduler->getMoves();
}
    
bool Helpers::isFakeMode() const
{
    static bool isFake = false;
    static bool asked = false;
    
    if (!asked) {
        asked = true;
        isFake = (_scheduler->getManager()
            ->BaseManager::parametersList()
            .paramStr("protocol").value 
            == "FakeProtocol");
    }
    
    return isFake;
}

bool Helpers::hasMove(const std::string& moveName) const
{
    return _scheduler->hasMove(moveName);
}

void Helpers::startMove(const std::string& moveName, double fade) const
{
    _scheduler->startMove(moveName, fade);
}

void Helpers::stopMove(const std::string& moveName, double fade) const
{
    _scheduler->stopMove(moveName, fade);
}

void Helpers::setAngle(const std::string& servo, float angle)
{
    _scheduler->getManager()->dev<RhAL::DXL>(servo)
        .goalPosition().writeValue(angle);
    _scheduler->getServices()->model->goalModel().get().setDOF(servo, 
        _scheduler->getManager()->dev<RhAL::DXL>(servo)
            .goalPosition().getWrittenValue()*M_PI/180.0);
}

void Helpers::setTorqueLimit(const std::string& servo, float torque)
{
    _scheduler->getManager()->dev<RhAL::DXL>(servo)
            .torqueLimit().writeValue(torque);
}
void Helpers::setPGain(const std::string& servo, float gain)
{
    _scheduler->getManager()->dev<RhAL::MX>(servo)
            .PGain().writeValue(gain);
}
void Helpers::setIGain(const std::string& servo, float gain)
{
    _scheduler->getManager()->dev<RhAL::MX>(servo)
            .IGain().writeValue(gain);
}
void Helpers::setDGain(const std::string& servo, float gain)
{
    _scheduler->getManager()->dev<RhAL::MX>(servo)
            .DGain().writeValue(gain);
}
void Helpers::setComplianceMarginCW(const std::string& servo, int gain)
{
    _scheduler->getManager()->dev<RhAL::RX>(servo)
            .complianceMarginCW().writeValue(gain);
}
void Helpers::setComplianceMarginCCW(const std::string& servo, int gain)
{
    _scheduler->getManager()->dev<RhAL::RX>(servo)
            .complianceMarginCCW().writeValue(gain);
}
void Helpers::setComplianceSlopeCW(const std::string& servo, int gain)
{
    _scheduler->getManager()->dev<RhAL::RX>(servo)
            .complianceSlopeCW().writeValue(gain);
}
void Helpers::setComplianceSlopeCCW(const std::string& servo, int gain)
{
    _scheduler->getManager()->dev<RhAL::RX>(servo)
            .complianceSlopeCCW().writeValue(gain);
}

bool Helpers::isMX(const std::string& servo)
{
    std::string type=_scheduler->getManager()->devTypeName(servo);

    return (type.substr(0,2) == "MX");
}

bool Helpers::isRX(const std::string& servo)
{
    std::string type=_scheduler->getManager()->devTypeName(servo);
    
    return (type.substr(0,2) == "RX");
}


float Helpers::getAngle(const std::string& servo)
{
    if (isFakeMode()) {
        return getServices()->model->goalModel()
            .get().getDOF(servo);
    } else {
        return _scheduler->getManager()->dev<RhAL::DXL>(servo)
                .position().readValue().value;
    }
}

float Helpers::getError(const std::string& servo)
{
    if (isFakeMode()) {
        return 0.0;
    } else {
        auto& dev = _scheduler->getManager()->dev<RhAL::DXL>(servo);
        return
                dev.position().readValue().value
                - dev.goalPosition().getWrittenValue();
    }
}

std::vector<std::string> Helpers::getServoNames()
{
    std::vector<std::string> servos;
    const auto& container = _scheduler->getManager()
                            ->devContainer<RhAL::DXL>();
    for (const auto& dev : container) {
        if (dev.second->isPresent()) {
            servos.push_back(dev.first);
        }
    }

    return servos;
}

float Helpers::getYaw()
{
    if (isFakeMode()) {
        return _scheduler->getServices()->model->goalModel().get().orientationYaw("trunk", "origin");
    } else {
        auto& gy85 = _scheduler->getManager()->dev<RhAL::GY85>("imu");
        return gy85.getYaw();
    }
}

float Helpers::getPitch()
{
    if (isFakeMode()) {
        return _scheduler->getServices()->model->goalModel().get().trunkSelfOrientation().y();
    } else {
        auto& gy85 = _scheduler->getManager()->dev<RhAL::GY85>("imu");
        return gy85.getPitch();
    }
}

float Helpers::getRoll()
{
    if (isFakeMode()) {
        return _scheduler->getServices()->model->goalModel().get().trunkSelfOrientation().x();
    } else {
        auto& gy85 = _scheduler->getManager()->dev<RhAL::GY85>("imu");
        return gy85.getRoll();
    }
}

float Helpers::getGyroYaw()
{
    if (isFakeMode()) {
        return _scheduler->getServices()->model->goalModel().get().orientationYaw("trunk", "origin");
    } else {
        auto& gy85 = _scheduler->getManager()->dev<RhAL::GY85>("imu");
        return gy85.getGyroYaw();
    }
}
    
float Helpers::getPressureWeight()
{
    auto& pressureLeft = _scheduler->getManager()->dev<RhAL::PressureSensor4>("left_pressure");
    auto& pressureRight = _scheduler->getManager()->dev<RhAL::PressureSensor4>("right_pressure");

    return pressureLeft.getWeight() + pressureRight.getWeight();
}

float Helpers::getPressureLeftRatio()
{
    if (isFakeMode()) {
        if (_scheduler->getServices()->model->goalModel().getSupportFoot() == Leph::HumanoidFixedModel::LeftSupportFoot) {
            return 1.0;
        } else {
            return 0.0;
        }
    } else {
        auto& pressureLeft = _scheduler->getManager()->dev<RhAL::PressureSensor4>("left_pressure");
        auto& pressureRight = _scheduler->getManager()->dev<RhAL::PressureSensor4>("right_pressure");

        float left = pressureLeft.getWeight();
        float right = pressureRight.getWeight();
        float total = left+right;

        if (total > 0) {
            return left/total;
        } else {
            return 0;
        }
    }
}

float Helpers::getPressureRightRatio()
{
    if (isFakeMode()) {
        if (_scheduler->getServices()->model->goalModel().getSupportFoot() == Leph::HumanoidFixedModel::RightSupportFoot) {
            return 1.0;
        } else {
            return 0.0;
        }
    } else {
        auto& pressureLeft = _scheduler->getManager()->dev<RhAL::PressureSensor4>("left_pressure");
        auto& pressureRight = _scheduler->getManager()->dev<RhAL::PressureSensor4>("right_pressure");

        float left = pressureLeft.getWeight();
        float right = pressureRight.getWeight();
        float total = left+right;

        if (total > 0) {
            return right/total;
        } else {
            return 0;
        }
    }
}

