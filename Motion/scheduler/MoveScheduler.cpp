#include <iomanip>
#include <iostream>
#include <fstream>
#include <sys/types.h>

#include "scheduler/MoveScheduler.h"
#include <rhoban_utils/timing/time_stamp.h>
#include <Devices/PressureSensor.hpp>
#include "Helpers.h"

using namespace std;
using namespace rhoban_utils;

// TODO: Remove this (could be an attribute of the scheduler), this
// was done to avoid a big re-compile
static Helpers helper;

MoveScheduler::MoveScheduler() :
        _isOver(false),
        _minTimeLoop(0.0),
        _avgTimeLoop(0.0),
        _maxTimeLoop(0.0),
        _minTimeFlush(0.0),
        _avgTimeFlush(0.0),
        _maxTimeFlush(0.0),
        _minTimeTickMoves(0.0),
        _avgTimeTickMoves(0.0),
        _maxTimeTickMoves(0.0),
        _minTimeTickServices(0.0),
        _avgTimeTickServices(0.0),
        _maxTimeTickServices(0.0),
        _manager(),
        _services(nullptr),
        _moves(nullptr),
        _binding(nullptr),
        _bind(nullptr)
{
    std::cout << "Initializing MoveScheduler" << std::endl;

    // Helper
    helper.setScheduler(this);

    //Initializing services
    _services = new Services(this);
    //Initializing moves
    _moves = new Moves(this);

    //Initializing RhAL
    std::cout << "Reading RhAL config" << std::endl;
    _manager.readConfig("rhal.json");
    _manager.setScheduleMode(true);
    //Initial scan
    std::cout << "Scanning the bus" << std::endl;
    _manager.scan();
    //Look for missing devices
    const auto& devs = _manager.devContainer();
    for (const auto& it : devs) {
        if (!it.second->isPresent()) {
            std::cout << "!!!!!!!!!!!! WARNING: Missing " 
                << it.first << std::endl;
        }
    }
    //Initialize RhIO RhAL Binding
    std::cout << "Starting RhIO binding" << std::endl;
    _binding = new RhAL::RhIOBinding(_manager);
    //Start Low level thread
    std::cout << "Starting RhAL Manager thread" << std::endl;
    _manager.startManagerThread();
    _manager.enableCooperativeThread();

    // TODO: This could be removed if Dynaban get fixed
    // Fixing Gain for Dynaban64
    for (auto &device : _manager.devContainer<RhAL::Dynaban64>()) {
        device.second->PGain().writeValue(30);
    }

    std::cout << "Initializing scheduler binding" << std::endl;
    //Binding move commands
    _bind = new RhIO::Bind("moves");
    _bind->bindFunc("moves", "List moves and states",
                    &MoveScheduler::cmdMoves, *this);
    _bind->bindFunc("start", "Starts a move, usage: start [move]",
                    &MoveScheduler::cmdStart, *this, {"init", "0.3"});
    _bind->bindFunc("stop", "Stops a move, usage: stop [move]",
                    &MoveScheduler::cmdStop, *this, {"all", "0.3"});
    _bind->bindFunc("init", "Initialize to zero",
                    &MoveScheduler::cmdInit, *this);
    _bind->bindFunc("em", "Emergency stop",
                    &MoveScheduler::cmdEm, *this);
    _bind->bindFunc("tare", "Alias for rhalTare",
                    &MoveScheduler::cmdTare, *this);


    _bind->bindFunc("gyroTare", "Alias for rhalGyroTare",
                    &MoveScheduler::cmdGyroTare, *this);
    _bind->bindFunc("stats", "MoveScheduler timing statistics",
                    &MoveScheduler::cmdStats, *this);
}

MoveScheduler::~MoveScheduler()
{
    std::cout << "Stopping RhIO Binding" << std::endl;
    delete _binding;
    std::cout << "Stopping Scheduler thread" << std::endl;
    _manager.disableCooperativeThread();
    std::cout << "Stopping RhAL Manager thread" << std::endl;
    _manager.stopManagerThread();
    delete _bind;
    delete _services;
    delete _moves;
}

bool MoveScheduler::hasMove(const string& name)
{
    return (_moves->hasMove(name) > 0);
}

void MoveScheduler::startMove(const string& name, double fade)
{
    getMove(name)->start(fade);
}

void MoveScheduler::stopMove(const string& name, double fade)
{
    if (name == "" || name == "all") {
        stopAllMoves(fade);
    } else {
        getMove(name)->stop(fade);
    }
}

void MoveScheduler::stopAllMoves(double fade)
{
    for (auto& entry : _moves->getAllMoves()) {
        getMove(entry.first)->stop(fade);
    }
}

void MoveScheduler::releaseServos()
{
    _manager.emergencyStop();
}

void MoveScheduler::execute()
{
    std::cout << "Starting main scheduler loop" << std::endl;
    bool initStats = true;
    TimeStamp startLoop = TimeStamp::now();
    TimeStamp stopLoop = TimeStamp::now();
    double durationLoop = 0.0;
    while (!_isOver) {
        startLoop = TimeStamp::now();

        //Wait next low level flush synchronisation
        TimeStamp startFlush = TimeStamp::now();
        _manager.waitNextFlush();
        TimeStamp stopFlush = TimeStamp::now();

        //Ticking services
        //(in defined orger by vector container)
        TimeStamp startTickServices = TimeStamp::now();
        for (const auto& service : _services->getAllServices()) {
            service.second->ElapseTick::tick();
        }
        TimeStamp stopTickServices = TimeStamp::now();

        //Ticking moves
        //(in defined orger by vector container)
        TimeStamp startTickMoves = TimeStamp::now();
        for (const auto& move : _moves->getAllMoves()) {
            // Ticking the move
            move.second->ElapseTick::tick();
        }
        TimeStamp stopTickMoves = TimeStamp::now();

        //Update statistics
        double durationFlush = diffMs(startFlush, stopFlush);
        double durationTickMoves = diffMs(startTickMoves, stopTickMoves);
        double durationTickServices = diffMs(startTickServices, stopTickServices);
        if (initStats) {
            _minTimeLoop = durationLoop;
            _maxTimeLoop = durationLoop;
            _avgTimeLoop = durationLoop;
            _minTimeFlush = durationFlush;
            _maxTimeFlush = durationFlush;
            _avgTimeFlush = durationFlush;
            _minTimeTickMoves = durationTickMoves;
            _maxTimeTickMoves = durationTickMoves;
            _avgTimeTickMoves = durationTickMoves;
            _minTimeTickServices = durationTickServices;
            _maxTimeTickServices = durationTickServices;
            _avgTimeTickServices = durationTickServices;
            initStats = false;
        } else {
            if (durationLoop < _minTimeLoop) _minTimeLoop = durationLoop;
            if (durationLoop > _maxTimeLoop) _maxTimeLoop = durationLoop;
            _avgTimeLoop = _avgTimeLoop*0.99 + durationLoop*0.01;
            if (durationFlush < _minTimeFlush) _minTimeFlush = durationFlush;
            if (durationFlush > _maxTimeFlush) _maxTimeFlush = durationFlush;
            _avgTimeFlush = _avgTimeFlush*0.99 + durationFlush*0.01;
            if (durationTickMoves < _minTimeTickMoves) _minTimeTickMoves = durationTickMoves;
            if (durationTickMoves > _maxTimeTickMoves) _maxTimeTickMoves = durationTickMoves;
            _avgTimeTickMoves = _avgTimeTickMoves*0.99 + durationTickMoves*0.01;
            if (durationTickServices < _minTimeTickServices) _minTimeTickServices = durationTickServices;
            if (durationTickServices > _maxTimeTickServices) _maxTimeTickServices = durationTickServices;
            _avgTimeTickServices = _avgTimeTickServices*0.99 + durationTickServices*0.01;
        }
        stopLoop = TimeStamp::now();
        durationLoop = diffMs(startLoop, stopLoop);
    }
    std::cout << "Stopping main scheduler loop" << std::endl;
}

Services* MoveScheduler::getServices()
{
    return _services;
}

const Services* MoveScheduler::getServices() const
{
    return _services;
}

bool MoveScheduler::isFakeMode()
{
    return getManager()->BaseManager::parametersList().paramStr("protocol").value == "FakeProtocol";
}

Moves* MoveScheduler::getMoves()
{
    return _moves;
}

Move* MoveScheduler::getMove(const std::string& name)
{
    return _moves->getMove(name);
}

const Move* MoveScheduler::getMove(const std::string& name) const
{
    return _moves->getMove(name);
}

RhAL::StandardManager* MoveScheduler::getManager()
{
    return &_manager;
}

void MoveScheduler::askQuit()
{
    _isOver = true;
}

std::string MoveScheduler::cmdMoves()
{
    std::string result;
    for (auto& entry : _moves->getAllMoves()) {
        Move* move = entry.second;
        char buffer[256];
        sprintf(buffer, "%-22s %-15s %-15s", move->getName().c_str(),
            move->getStatus().c_str(), move->getError().c_str());
        result += buffer;
        if (move->hasStats) {
            sprintf(buffer, " [min: %.1fms, avg: %.1fms, max: %.1fms]",
                move->minTime, move->avgTime, move->maxTime);
            result += buffer;
        }
        result += "\n";
    }

    return result;
}

std::string MoveScheduler::cmdStart(std::string name, double fade)
{
    if (name == "init") {
        return cmdInit();
    } else {
        try {
            getMove(name)->start(fade);
            return "Started";
        } catch (std::runtime_error err) {
            return std::string("Error: ") + err.what();
        }
    }
}

std::string MoveScheduler::cmdInit()
{
    stopAllMoves(-1);
    _manager.exitEmergencyState(); //torque enable (useful for dynaban)
    _binding->cmdInit({});

    return "OK";
}

std::string MoveScheduler::cmdStop(std::string name, double fade)
{
    try {
        stopMove(name, fade);
        return "Stopped";
    } catch (std::runtime_error err) {
        return std::string("Error: ") + err.what();
    }
}

std::string MoveScheduler::cmdEm()
{
    auto hasAnyMove = [this]() -> bool {
        for (auto &move : _moves->getAllMoves()) {
            if (move.second->isRunning()) {
                return true;
            }
        }
        return false;
    };
    while (hasAnyMove()) {
        stopAllMoves(-1);
    }
    for (int k=0; k<3; k++) {
        releaseServos();
        _manager.waitNextFlush();
    }

    return "Killing all moves, releasing servos";
}

std::string MoveScheduler::cmdTare()
{
    return _binding->cmdTare({});
}

std::string MoveScheduler::cmdGyroTare()
{
    return _binding->cmdGyroTare({});
}

std::string MoveScheduler::cmdStats()
{
    std::ostringstream os;
    os << "MoveScheduler Statistics" << std::endl;
    os << std::setfill(' ') << std::setw(20) << "Name"
        << std::setfill(' ') << std::setw(15) << "min time"
        << std::setfill(' ') << std::setw(15) << "avg time"
        << std::setfill(' ') << std::setw(15) << "ratio"
        << std::setfill(' ') << std::setw(15) << "max time"
        << std::endl;
    os << std::setfill(' ') << std::setw(20) << "Main Scheduler Loop:"
        << std::setfill(' ') << std::setw(15) << to_string(_minTimeLoop) + "ms"
        << std::setfill(' ') << std::setw(15) << to_string(_avgTimeLoop) + "ms"
        << std::setfill(' ') << std::setw(15) << "100%"
        << std::setfill(' ') << std::setw(15) << to_string(_maxTimeLoop) + "ms"
        << std::endl;
    os << std::setfill(' ') << std::setw(20) << "Manager Wait Flush:"
        << std::setfill(' ') << std::setw(15) << to_string(_minTimeFlush) + "ms"
        << std::setfill(' ') << std::setw(15) << to_string(_avgTimeFlush) + "ms"
        << std::setfill(' ') << std::setw(15) << to_string(_avgTimeFlush/_avgTimeLoop*100.0) + "%"
        << std::setfill(' ') << std::setw(15) << to_string(_maxTimeFlush) + "ms"
        << std::endl;
    os << std::setfill(' ') << std::setw(20) << "Tick Services:"
        << std::setfill(' ') << std::setw(15) << to_string(_minTimeTickServices) + "ms"
        << std::setfill(' ') << std::setw(15) << to_string(_avgTimeTickServices) + "ms"
        << std::setfill(' ') << std::setw(15) << to_string(_avgTimeTickServices/_avgTimeLoop*100.0) + "%"
        << std::setfill(' ') << std::setw(15) << to_string(_maxTimeTickServices) + "ms"
        << std::endl;
    os << std::setfill(' ') << std::setw(20) << "Tick Moves:"
        << std::setfill(' ') << std::setw(15) << to_string(_minTimeTickMoves) + "ms"
        << std::setfill(' ') << std::setw(15) << to_string(_avgTimeTickMoves) + "ms"
        << std::setfill(' ') << std::setw(15) << to_string(_avgTimeTickMoves/_avgTimeLoop*100.0) + "%"
        << std::setfill(' ') << std::setw(15) << to_string(_maxTimeTickMoves) + "ms"
        << std::endl;
    os << "Timing Services:" << std::endl;
    for (auto& entry : _services->getAllServices()) {
        if (entry.second->hasStats) {
            os << std::setfill(' ') << std::setw(20) << entry.first + ":"
                << std::setfill(' ') << std::setw(15) << to_string(entry.second->minTime) + "ms"
                << std::setfill(' ') << std::setw(15) << to_string(entry.second->avgTime) + "ms"
                << std::setfill(' ') << std::setw(15) << to_string(entry.second->avgTime/_avgTimeLoop*100.0) + "%"
                << std::setfill(' ') << std::setw(15) << to_string(entry.second->maxTime) + "ms"
                << std::endl;
        }
    }
    os << "Timing Moves:" << std::endl;
    for (auto& entry : _moves->getAllMoves()) {
        if (entry.second->hasStats) {
            os << std::setfill(' ') << std::setw(20) << entry.first + ":"
                << std::setfill(' ') << std::setw(15) << to_string(entry.second->minTime) + "ms"
                << std::setfill(' ') << std::setw(15) << to_string(entry.second->avgTime) + "ms"
                << std::setfill(' ') << std::setw(15) << to_string(entry.second->avgTime/_avgTimeLoop*100.0) + "%"
                << std::setfill(' ') << std::setw(15) << to_string(entry.second->maxTime) + "ms"
                << std::endl;
        }
    }

    return os.str();
}

