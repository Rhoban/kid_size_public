#include "FieldObservation.hpp"

#include "Field/Field.hpp"

#include <robocup_referee/constants.h>

using namespace rhoban_geometry;
using namespace robocup_referee;

namespace Vision {
namespace Localisation {

/// Giving zero scores to particles is a huge issue
double FieldObservation::pError = 0.001;

FieldObservation::FieldObservation(bool isGoalKeeper):isGoalKeeper(isGoalKeeper) {}

double FieldObservation::potential(const FieldPosition &p) const {
  Point pos = p.getRobotPosition();
  // Refusing entirely to be behind the goal because it can cause weird
  // approaches
  double dx = abs(pos.x) - Constants::field.fieldLength / 2;
  // Accepting to go slightly out of the field laterally
  double dy =
    abs(pos.y) - (Constants::field.fieldWidth / 2 - Constants::field.borderStripWidth);
  if (dx > 0 || dy > 0) {
    return pError;
  }
  return 1;
}

double FieldObservation::getMinScore() const { return pError; }
}
}
