#pragma once

#include <cstdlib>
#include <list>

#include "rhoban_geometry/point.h"

namespace Vision {
namespace Localisation {
class SpeedEstimator {
private:
  typedef std::pair<double, rhoban_geometry::Point> TimedPosition;
  // MEMORY
  /// Positions are stored in world referential
  std::list<TimedPosition> positions;
  /// Qualities of the observations
  std::list<double> qualities;
  /// Maximal number of elements stored
  unsigned int memorySize;

  // Discount rate for weights
  double disc;

  // Stored results
  rhoban_geometry::Point speed, qSpeed;

public:
  SpeedEstimator();

  // Insert the value if it is not equivalent to the last on
  void update(double ts, const rhoban_geometry::Point &p, double quality = 1);

  rhoban_geometry::Point getSpeed(); // in m/s, in world referential
  double getQuality();
  rhoban_geometry::Point getQualities(); // q are separate for x and y

  rhoban_geometry::Point getUsableSpeed();
};

}
}
