#include <stdexcept>
#include <rhoban_utils/timing/time_stamp.h>
#include <rhoban_utils/logging/logger.h>
#include "services/Services.h"
#include "scheduler/MoveScheduler.h"
#include "services/RobotModelService.h"
#include "services/LocalisationService.h"
#include "Helpers.h"
#include <Devices/GY85.hpp>

using namespace rhoban_utils;

static rhoban_utils::Logger out("helpers");

bool Helpers::isPython = false;
bool Helpers::fakeIMU = false;
double Helpers::fakeYaw = 0.0;
double Helpers::fakePitch = 0.0;
double Helpers::fakeRoll = 0.0;

bool Helpers::fakePressure = false;
double Helpers::pressureLeftX, Helpers::pressureLeftY, Helpers::pressureLeftWeight;
double Helpers::pressureRightX, Helpers::pressureRightY, Helpers::pressureRightWeight;

Helpers::Helpers() : _scheduler(nullptr)
{
}

void Helpers::setScheduler(MoveScheduler* scheduler)
{
  _scheduler = scheduler;
  if (_scheduler == nullptr)
  {
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

  if (!asked)
  {
    asked = true;
    isFake = (_scheduler->getManager()->BaseManager::parametersList().paramStr("protocol").value == "FakeProtocol");
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
  _scheduler->getManager()->dev<RhAL::DXL>(servo).goalPosition().writeValue(angle);
}

void Helpers::setTorqueLimit(const std::string& servo, float torque)
{
  _scheduler->getManager()->dev<RhAL::DXL>(servo).torqueLimit().writeValue(torque);
}
void Helpers::setPGain(const std::string& servo, float gain)
{
  _scheduler->getManager()->dev<RhAL::MX>(servo).PGain().writeValue(gain);
}
void Helpers::setIGain(const std::string& servo, float gain)
{
  _scheduler->getManager()->dev<RhAL::MX>(servo).IGain().writeValue(gain);
}
void Helpers::setDGain(const std::string& servo, float gain)
{
  _scheduler->getManager()->dev<RhAL::MX>(servo).DGain().writeValue(gain);
}
void Helpers::setComplianceMarginCW(const std::string& servo, int gain)
{
  _scheduler->getManager()->dev<RhAL::RX>(servo).complianceMarginCW().writeValue(gain);
}
void Helpers::setComplianceMarginCCW(const std::string& servo, int gain)
{
  _scheduler->getManager()->dev<RhAL::RX>(servo).complianceMarginCCW().writeValue(gain);
}
void Helpers::setComplianceSlopeCW(const std::string& servo, int gain)
{
  _scheduler->getManager()->dev<RhAL::RX>(servo).complianceSlopeCW().writeValue(gain);
}
void Helpers::setComplianceSlopeCCW(const std::string& servo, int gain)
{
  _scheduler->getManager()->dev<RhAL::RX>(servo).complianceSlopeCCW().writeValue(gain);
}

bool Helpers::isMX(const std::string& servo)
{
  std::string type = _scheduler->getManager()->devTypeName(servo);

  return (type.substr(0, 2) == "MX");
}

bool Helpers::isRX(const std::string& servo)
{
  std::string type = _scheduler->getManager()->devTypeName(servo);

  return (type.substr(0, 2) == "RX");
}

void Helpers::lockScheduler()
{
  _scheduler->mutex.lock();
}

void Helpers::unlockScheduler()
{
  _scheduler->mutex.unlock();
}

void Helpers::setSchedulerClock(double value)
{
  _scheduler->setManualClock(value);
}

float Helpers::getAngle(const std::string& servo)
{
  if (isFakeMode())
  {
    return getServices()->robotModel->model.getDof(servo);
  }
  else
  {
    return _scheduler->getManager()->dev<RhAL::DXL>(servo).position().readValue().value;
  }
}

float Helpers::getError(const std::string& servo)
{
  if (isFakeMode())
  {
    return 0.0;
  }
  else
  {
    auto& dev = _scheduler->getManager()->dev<RhAL::DXL>(servo);
    return dev.position().readValue().value - dev.goalPosition().getWrittenValue();
  }
}

std::vector<std::string> Helpers::getServoNames()
{
  std::vector<std::string> servos;
  const auto& container = _scheduler->getManager()->devContainer<RhAL::DXL>();
  for (const auto& dev : container)
  {
    if (dev.second->isPresent())
    {
      servos.push_back(dev.first);
    }
  }

  return servos;
}

void Helpers::setFakeIMU(double yaw, double pitch, double roll)
{
  Helpers::fakeYaw = yaw;
  Helpers::fakePitch = pitch;
  Helpers::fakeRoll = roll;
  Helpers::fakeIMU = true;
}

void Helpers::setFakePosition(double x, double y, double theta)
{
  auto loc = _scheduler->getServices()->localisation;
  loc->cmdMoveOnField(x, y, theta);
}

void Helpers::setFakeBallPosition(double x, double y)
{
  auto loc = _scheduler->getServices()->localisation;
  loc->cmdFakeBall(x, y);
}

float Helpers::getPitch()
{
  if (Helpers::fakeIMU)
  {
    return Helpers::fakePitch;
  }

  if (isFakeMode())
  {
    // XXX: Do we care about having a pitch in fake mode ?
    return 0.0;
  }
  else
  {
    auto& gy85 = _scheduler->getManager()->dev<RhAL::GY85>("imu");
    return gy85.getPitch();
  }
}

float Helpers::getRoll()
{
  if (Helpers::fakeIMU)
  {
    return Helpers::fakeRoll;
  }

  if (isFakeMode())
  {
    // XXX: Do we care about having a roll in fake mode ?
    return 0.0;
  }
  else
  {
    auto& gy85 = _scheduler->getManager()->dev<RhAL::GY85>("imu");
    return gy85.getRoll();
  }
}

float Helpers::getGyroYaw()
{
  if (Helpers::fakeIMU)
  {
    return Helpers::fakeYaw;
  }

  if (isFakeMode())
  {
    return rhoban::frameYaw(getServices()->robotModel->model.selfToWorld().rotation());
  }
  else
  {
    auto& gy85 = _scheduler->getManager()->dev<RhAL::GY85>("imu");
    return gy85.getGyroYaw();
  }
}

void Helpers::updatePressure()
{
  if (Helpers::fakePressure)
  {
    return;
  }

  if (isFakeMode())
  {
    if (_scheduler->getServices()->robotModel->model.supportFoot == rhoban::HumanoidModel::Left)
    {
      pressureLeftWeight = 1.0;
      pressureRightWeight = 0.0;
    }
    else
    {
      pressureLeftWeight = 0.0;
      pressureRightWeight = 1.0;
    }
  }
  else
  {
    auto& pressureLeft = _scheduler->getManager()->dev<RhAL::PressureSensor4>("left_pressure");
    auto& pressureRight = _scheduler->getManager()->dev<RhAL::PressureSensor4>("right_pressure");

    pressureLeftWeight = pressureLeft.getWeight();
    pressureLeftX = pressureLeft.getX();
    pressureLeftY = pressureLeft.getY();
    pressureRightWeight = pressureRight.getWeight();
    pressureRightX = pressureRight.getX();
    pressureRightY = pressureRight.getY();
  }
}

void Helpers::setFakePressure(double left_x, double left_y, double left_weight, double right_x, double right_y,
                              double right_weight)
{
  fakePressure = true;
  pressureLeftX = left_x;
  pressureLeftX = left_y;
  pressureLeftWeight = left_weight;
  pressureRightX = right_x;
  pressureRightX = right_y;
  pressureRightWeight = right_weight;
}

float Helpers::getPressureWeight()
{
  updatePressure();

  return pressureLeftWeight + pressureRightWeight;
}

float Helpers::getPressureLeftRatio()
{
  updatePressure();

  double total = pressureLeftWeight + pressureRightWeight;
  if (total > 0)
  {
    return pressureLeftWeight / total;
  }
  else
  {
    return 0;
  }
}

float Helpers::getPressureRightRatio()
{
  updatePressure();

  double total = pressureLeftWeight + pressureRightWeight;
  if (total > 0)
  {
    return pressureRightWeight / total;
  }
  else
  {
    return 0;
  }
}

float Helpers::getPressureX()
{
  updatePressure();

  double total = pressureLeftWeight + pressureRightWeight;
  if (total > 0)
  {
    return (pressureLeftX * pressureLeftWeight + pressureRightX * pressureRightWeight) / total;
  }
  else
  {
    return 0;
  }
}

float Helpers::getPressureY()
{
  updatePressure();

  double total = pressureLeftWeight + pressureRightWeight;
  if (total > 0)
  {
    return ((pressureLeftY + 0.07) * pressureLeftWeight + (pressureRightY - 0.07) * pressureRightWeight) / total;
  }
  else
  {
    return 0;
  }
}
