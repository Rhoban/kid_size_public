#include "FieldObservation.hpp"

#include <robocup_referee/constants.h>

using namespace rhoban_geometry;
using namespace robocup_referee;

namespace Vision
{
namespace Localisation
{
/// Giving zero scores to particles is a huge issue
double FieldObservation::pError = 0.001;

FieldObservation::FieldObservation(bool isGoalKeeper) : isGoalKeeper(isGoalKeeper)
{
}

double FieldObservation::potential(const FieldPosition& p) const
{
  Point pos = p.getRobotPosition();
  // Refusing entirely to be behind the goal because it can cause weird
  // approaches
  // TODO: this should not be done, approaches should take it into account
  double dx = abs(pos.x) - Constants::field.field_length / 2;
  // Accepting to go slightly out of the field laterally
  double dy = abs(pos.y) - (Constants::field.field_width / 2 - Constants::field.border_strip_width_y);
  if (dx > 0 || dy > 0)
  {
    return pError;
  }
  return 1;
}

double FieldObservation::getMinScore() const
{
  return pError;
}
}  // namespace Localisation
}  // namespace Vision
