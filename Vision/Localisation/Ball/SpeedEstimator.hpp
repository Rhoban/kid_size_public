#pragma once

#include <cstdlib>
#include <list>

#include "rhoban_utils/timing/time_stamp.h"
#include "rhoban_geometry/point.h"

namespace RhIO {
class Bind;
}

namespace Vision {
namespace Localisation {
class SpeedEstimator {
public:
  SpeedEstimator();

  // Insert the value if it is not equivalent to the last on
  void update(const rhoban_utils::TimeStamp & ts,
              const rhoban_geometry::Point &p, double quality = 1);

  rhoban_geometry::Point getSpeed(); // in m/s, in world referential
  double getQuality();
  rhoban_geometry::Point getQualities(); // q are separate for x and y

  rhoban_geometry::Point getUsableSpeed();

private:
  typedef std::pair<rhoban_utils::TimeStamp, rhoban_geometry::Point> TimedPosition;
  typedef std::pair<rhoban_geometry::Point, double> WeightedSpeed;

  RhIO::Bind * bind;

  /// Positions are stored in world referential (most recent entry front)
  std::deque<TimedPosition> positions;
  /// Qualities of the observations (most recent entry front)
  std::deque<double> qualities;
  /// Maximal number of elements stored
  int memory_size;

  /// Minimal spacing between to entries to compute their speed
  double min_dt;

  /// Maximal time difference between the first and the last entry
  double max_dt;

  /// Discount rate for weights
  double disc;

  /// Tolerance for quality
  double flat_tol;

  // Stored results
  rhoban_geometry::Point speed, qSpeed;

  void initBinding();

  /// Remove old entries based on max_dt and memory_size
  void cleanOldEntries();

  /// Compute speeds candidates based on memory content and min_dt
  std::vector<WeightedSpeed> getWeightedSpeeds() const;
};

}
}
