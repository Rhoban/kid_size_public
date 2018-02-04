#include "FieldObservation.hpp"

#include "Field/Field.hpp"

using namespace rhoban_geometry;

namespace Vision {
namespace Localisation {

/// Giving zero scores to particles is a huge issue
double FieldObservation::pError = 0.001;

FieldObservation::FieldObservation(bool isGoalKeeper):isGoalKeeper(isGoalKeeper) {}

double FieldObservation::potential(const FieldPosition &p) const {
  Point pos = p.getRobotPosition();
  // Refusing entirely to be behind the goal because it can cause weird
  // approaches
  double dx = abs(pos.x) - Field::Field::fieldLength / 2;
  // Accepting to go slightly out of the field laterally
  double dy =
      abs(pos.y) - Field::Field::fieldWidth / 2 - Field::Field::borderStrip;
  if (dx > 0 || dy > 0) {
    // std::cout << "FieldObservation: score = 0: " << pos.x << "," << pos.y <<
    // std::endl;
    return pError;
  }
  //  if ((isGoalKeeper) && (pos.x>0)) should not be done here...
  //return pError;
  return 1;
}

double FieldObservation::getMinScore() const { return pError; }
}
}
