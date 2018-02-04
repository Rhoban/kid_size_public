#pragma once

#include "CameraState/CameraState.hpp"
#include "Localisation/RadarFilterPoint.hpp"

namespace Vision
{
namespace Localisation
{

class RobotFilter : public RadarFilterPoint
{
    public:
        RobotFilter(Utils::CameraState *cs);

        virtual bool defaultNormalizeSum();
};

}
}
