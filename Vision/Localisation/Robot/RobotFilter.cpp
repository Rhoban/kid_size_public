#include "RobotFilter.hpp"

namespace Vision
{
namespace Localisation
{
        
RobotFilter::RobotFilter(Utils::CameraState *cs)
    : RadarFilterPoint(cs)
{
}

bool RobotFilter::defaultNormalizeSum()
{
    return false;
}

}
}
