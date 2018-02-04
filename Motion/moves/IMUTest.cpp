#include <math.h>
#include "IMUTest.h"
        
IMUTest::IMUTest()
{
    Move::initializeBinding();
    t = 0.0;
    bind->bindNew("freq", freq, RhIO::Bind::PullOnly)
        ->defaultValue(1.0);
    bind->bindNew("amplitude", amplitude, RhIO::Bind::PullOnly)
        ->defaultValue(5.0);
}

std::string IMUTest::getName()
{
    return "imutest";
}

void IMUTest::onStart()
{
}

void IMUTest::step(float elapsed)
{
    t += elapsed;
    bind->pull();
    float v = sin(t*M_PI*2*freq)*amplitude;
    setAngle("left_hip_pitch", v);
    setAngle("right_hip_pitch", v);
    printf("%g %g\n", getAngle("left_hip_pitch"), getPitch());
}
