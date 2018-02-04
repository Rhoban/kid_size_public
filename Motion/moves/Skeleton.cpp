#include <math.h>
#include "Skeleton.h"

std::string Skeleton::getName()
{
    return "skeleton";
}

void Skeleton::onStart()
{
}

void Skeleton::step(float elapsed)
{
    setAngle("head_pitch", getSensorValue("Pitch"));
}
