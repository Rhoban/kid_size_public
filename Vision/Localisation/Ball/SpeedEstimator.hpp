#pragma once

#include <rhoban_utils/timing/time_stamp.h>

#include <Eigen/Core>

#include <vector>
#include <cstdlib>
#include <deque>

namespace RhIO
{
class Bind;
}

namespace Vision
{
namespace Localisation
{
class SpeedEstimator
{
public:
  SpeedEstimator();

  // Insert the value if it is not equivalent to the last on
  void update(const rhoban_utils::TimeStamp& ts, const Eigen::Vector2d& pos);

  Eigen::Vector2d getSpeed();  // in m/s, in world referential
  double getQuality();

  Eigen::Vector2d getUsableSpeed();

private:
  typedef std::pair<rhoban_utils::TimeStamp, Eigen::Vector2d> TimedPosition;
  typedef std::pair<Eigen::Vector2d, double> WeightedSpeed;

  RhIO::Bind* bind;

  /// Positions are stored in world referential (most recent entry front)
  std::deque<TimedPosition> positions;
  /// Maximal number of elements stored
  int memory_size;

  /// Minimal spacing between to entries to compute their speed
  double min_dt;

  /// Maximal time difference between the first and the last entry
  double max_dt;

  /// Discount rate for weights
  double disc;

  /// A factor used to score the quality of speed, if deviation on speed is
  /// higher than max_speed_dev, speed_quality will be 0
  double max_speed_dev;

  /// Theoric limit for ball speed, entries with a higher speed will simply be
  /// ignored
  double max_speed;

  /// Stored results
  Eigen::Vector2d speed;
  Eigen::Vector2d usable_speed;

  /// Quality of speed estimation
  double speed_quality;

  /// Verbosity of debug output (0 -> no output)
  int debug_level;

  /// The speeds measured from couples
  std::vector<WeightedSpeed> measured_speeds;

  void initBinding();

  /// Remove old entries based on max_dt and memory_size
  void cleanOldEntries();

  /// Compute speeds candidates based on memory content and min_dt
  void updateMeasuredSpeeds();
};

}  // namespace Localisation
}  // namespace Vision
