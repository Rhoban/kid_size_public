#include <iostream>
#include <RhIO.hpp>
#include <robocup_referee/constants.h>
#include <pybind11/pybind11.h>
#include <rhoban_utils/angle.h>
#include <pybind11/stl.h>
#include "scheduler/MoveScheduler.h"
#include "scheduler/Helpers.h"

using namespace std;
namespace py = pybind11;

std::thread* moveSchedulerThread;
Helpers* execute()
{
  Helpers::isPython = true;

  RhIO::start(RhIO::ServersPortBase);
  robocup_referee::Constants::field.loadFile("field.json");
  RhIO::Root.load("rhio");

  MoveScheduler* scheduler = NULL;
  Helpers* helpers = new Helpers();

  // Running a move scheduler thread
  moveSchedulerThread = new std::thread([&scheduler, &helpers]() {
    try
    {
      scheduler = new MoveScheduler();
      helpers->setScheduler(scheduler);
      RhIO::Root.setBool("/decision/isBallQualityGood", true);
      RhIO::Root.setBool("/decision/isFieldQualityGood", true);
      scheduler->execute(true);
    }
    catch (const std::runtime_error& exc)
    {
      cout << "ERROR: Server died with runtime exception " << endl << exc.what() << endl;
    }
    catch (const std::logic_error& exc)
    {
      cout << "ERROR: Server died with logic exception " << endl << exc.what() << endl;
    }
    catch (const std::string& exc)
    {
      cout << "ERROR: Server died with string exception " << endl << exc << endl;
    }

    scheduler->askQuit();
  });

  // Waiting for the helpers to be set
  while (helpers->getScheduler() == NULL)
  {
    usleep(1000);
  }

  return helpers;
}

class PyRhio
{
public:
  PyRhio() : bind("python")
  {
    bind.bindFunc("pyFriction", "", &PyRhio::cmdFriction, *this);
    bind.bindFunc("pyPos", "", &PyRhio::cmdPos, *this);
    bind.bindFunc("pyBall", "", &PyRhio::cmdBall, *this);
    bind.bindFunc("pyReset", "", &PyRhio::cmdReset, *this);
  }
  RhIO::Bind bind;

  std::string cmdFriction(double lateral, double spinning, double rolling)
  {
    hasFriction = true;
    lateralFriction = lateral;
    spinningFriction = spinning;
    rollingFriction = rolling;

    return "Set friction";
  }
  bool hasFriction = false;
  double lateralFriction = 0.75;
  double spinningFriction = 0.1;
  double rollingFriction = 0.1;

  std::string cmdPos(double x, double y, double theta=0)
  {
    hasPos = true;
    posX = x;
    posY = y;
    posTheta = rhoban_utils::deg2rad(theta);
    return "OK";
  }
  bool hasPos = false;
  double posX, posY, posTheta;

  std::string cmdBall(double x, double y)
  {
    hasBall = true;
    ballX = x;
    ballY = y;
    return "OK";
  }
  bool hasBall = false;
  double ballX, ballY;

  std::string cmdReset()
  {
    cmdPos(0, 0, 0);
    cmdBall(0.25, -0.05);
    return "OK";
  }
};

PYBIND11_MODULE(rhoban, m)
{
  // Binding execute method
  m.def("execute", &execute);

  // Binding helpers class
  py::class_<Helpers>(m, "Helpers")
    .def(py::init<>())
    .def("setScheduler", &Helpers::setScheduler)
    .def("lockScheduler", &Helpers::lockScheduler)
    .def("unlockScheduler", &Helpers::unlockScheduler)
    .def("getAngle", &Helpers::getAngle)
    .def("setSchedulerClock", &Helpers::setSchedulerClock)
    .def("setFakeIMU", &Helpers::setFakeIMU)
    .def("setFakePosition", &Helpers::setFakePosition)
    .def("setFakeBallPosition", &Helpers::setFakeBallPosition)
  ;

  py::class_<PyRhio>(m, "PyRhio")
    .def(py::init<>())

    .def_readwrite("hasFriction", &PyRhio::hasFriction)
    .def_readwrite("lateralFriction", &PyRhio::lateralFriction)
    .def_readwrite("spinningFriction", &PyRhio::spinningFriction)
    .def_readwrite("rollingFriction", &PyRhio::rollingFriction)

    .def_readwrite("hasPos", &PyRhio::hasPos)
    .def_readwrite("posX", &PyRhio::posX)
    .def_readwrite("posY", &PyRhio::posY)
    .def_readwrite("posTheta", &PyRhio::posTheta)

    .def_readwrite("hasBall", &PyRhio::hasBall)
    .def_readwrite("ballX", &PyRhio::ballX)
    .def_readwrite("ballY", &PyRhio::ballY)
  ;
}