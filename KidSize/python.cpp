#include <iostream>
#include <RhIO.hpp>
#include <robocup_referee/constants.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "scheduler/MoveScheduler.h"
#include "scheduler/Helpers.h"

using namespace std;
namespace py = pybind11;

std::thread* moveSchedulerThread;
Helpers* execute()
{
  RhIO::start(RhIO::ServersPortBase);
  robocup_referee::Constants::field.loadFile("field.json");
  RhIO::Root.load("rhio");
  MoveScheduler* scheduler = NULL;
  Helpers* helpers = new Helpers();

  moveSchedulerThread = new std::thread([&scheduler, &helpers]() {
    try
    {
      scheduler = new MoveScheduler();
      helpers->setScheduler(scheduler);
      scheduler->execute();
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

  while (helpers->getScheduler() == NULL) {
    usleep(1000);
  }

  return helpers;
}

PYBIND11_MODULE(rhoban, m)
{
  m.def("execute", &execute);

  py::class_<MoveScheduler>(m, "MoveScheduler").def(py::init<>()).def("execute", &MoveScheduler::execute);

  py::class_<Helpers>(m, "Helpers")
      .def(py::init<>())
      .def("setScheduler", &Helpers::setScheduler)
      .def("lockScheduler", &Helpers::lockScheduler)
      .def("unlockScheduler", &Helpers::unlockScheduler)
      .def("getAngle", &Helpers::getAngle);
}