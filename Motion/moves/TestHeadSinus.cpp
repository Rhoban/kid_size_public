#include <cmath>
#include "moves/TestHeadSinus.hpp"
        
TestHeadSinus::TestHeadSinus() :
    Move(),
    _t(0.0),
    _freqTilt(0.0),
    _freqPan(0.0),
    _amplTilt(10.0),
    _amplPan(10.0)
{
    Move::initializeBinding();
    Move::bind->bindNew("freqTilt", _freqTilt, RhIO::Bind::PullOnly)
        ->defaultValue(4.0);
    Move::bind->bindNew("freqPan", _freqPan, RhIO::Bind::PullOnly)
        ->defaultValue(3.0);
    Move::bind->bindNew("amplTilt", _amplTilt, RhIO::Bind::PullOnly)
        ->defaultValue(15.0);
    Move::bind->bindNew("amplPan", _amplPan, RhIO::Bind::PullOnly)
        ->defaultValue(10.0);
}

std::string TestHeadSinus::getName()
{
    return "testHeadSinus";
}
void TestHeadSinus::onStart()
{
    _t = 0.0;
}
void TestHeadSinus::step(float elapsed)
{
    Move::bind->pull();

    Helpers::setAngle("head_pitch", _amplTilt*sin(_freqTilt*_t));
    Helpers::setAngle("head_yaw", _amplPan*sin(_freqPan*_t));
    _t += elapsed;

    Move::bind->push();
}

