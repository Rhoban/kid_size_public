#include <rhoban_utils/timing/time_stamp.h>
#include "scheduler/MoveScheduler.h"
#include "moves/Move.h"
#include "services/Services.h"

using namespace rhoban_utils;
using namespace std;

Move::Move() :
    Helpers(),
    ElapseTick(),
    bind(nullptr),
    _isRunning(false),
    _error()
{
}

Move::~Move()
{
    if (bind != nullptr) {
        delete bind;
    }
}

void Move::onStart()
{
}

void Move::onStop()
{
}

bool Move::isRunning() const
{
    return _isRunning;
}

string Move::getStatus() const
{
    if (_isRunning) {
        switch (_smoothingState) {
            case SMOOTHING_IN:
                return "fading-in";
                break;
            case SMOOTHING_RUNNING:
                return "running";
                break;
            case SMOOTHING_OUT:
                return "fading-out";
                break;
        }
    }
    return "stopped";
}

string Move::getError() const
{
    return _error;
}

void Move::start(double fade)
{
    if (!isRunning()) {
        if (fade > 0.0) {
            _smoothingDuration = _smoothingRemaining = fade;
            _smoothingState = SMOOTHING_IN;
        } else {
            _smoothing = 1.0;
            _smoothingState = SMOOTHING_RUNNING;
        }

        _isRunning = true;
        onStart();
    }
}

void Move::stop(double fade)
{
    if (isRunning()) {
        if (fade > 0.0) {
            _smoothingDuration = _smoothingRemaining = fade;
            _smoothingState = SMOOTHING_OUT;
        } else {
            onStop();
            _isRunning = false;
            _smoothing = 0.0;
        }
    }
}
        
bool Move::isFading() const
{
    return _smoothingState == SMOOTHING_IN || _smoothingState == SMOOTHING_OUT;
}

bool Move::tick(double elapsed)
{
    if (isRunning()) {
        if (_smoothingState != SMOOTHING_RUNNING) {
            if (_smoothingState == SMOOTHING_IN) {
                _smoothing = 1.0-_smoothingRemaining/_smoothingDuration;
                if (_smoothing >= 1.0) {
                    _smoothing = 1.0;
                    _smoothingState = SMOOTHING_RUNNING;
                }
            } else if (_smoothingState == SMOOTHING_OUT) {
	      _smoothing = _smoothingRemaining/_smoothingDuration;
                if (_smoothing <= 0.0) {
                    _smoothing = 0.0;
                    _isRunning = false;
                }
            }
            _smoothingRemaining -= elapsed;
        }

        try {
            step(elapsed);
        } catch (const std::runtime_error & exc) {
            _error = "Exception runtime_error: "
                     + string(exc.what());
        } catch (const std::logic_error & exc) {
            _error = "Exception logic_error: "
                     + string(exc.what());
        } catch (const std::string & exc) {
            _error = "Exception string: "
                     + exc;
        } catch (...) {
            _error = "Unknown exception catched";
        }

        if (!isRunning()) {
            onStop();
        }
    }

    return isRunning();
}

void Move::initializeBinding()
{
    if (bind != nullptr) {
        throw std::logic_error(
            "Move already initialize: "
            + this->getName());
    }

    bind = new RhIO::Bind(
        "moves/" + this->getName());

    bind->bindFunc(
        this->getName(),
        "Starts the move " + this->getName(),
        &Move::cmdStart, *this);
}

std::string Move::cmdStart()
{
    start(0.3);
    std::stringstream ss;
    ss << "Started move " << this->getName() << ".";

    return ss.str();
}


void Move::setAngle(const std::string& servo, float angle)
{
    this->Helpers::setAngle(servo, angle*_smoothing);
}
