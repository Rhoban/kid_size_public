#include "Source.hpp"

#include <rhoban_utils/timing/time_stamp.h>

namespace Vision
{
namespace Filters
{


int64_t Source::getClockOffset() const
{
  return rhoban_utils::getSteadyClockOffset();
}

}  // namespace Filters
}  // namespace Vision
