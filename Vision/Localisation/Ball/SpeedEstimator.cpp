#include "SpeedEstimator.hpp"
#include "rhoban_utils/timing/time_stamp.h"
#include "rhoban_utils/logging/logger.h"

#include <RhIO.hpp>

#include <cmath>

using ::rhoban_utils::TimeStamp;
using namespace std;

static rhoban_utils::Logger logger("SpeedEstimator");

namespace Vision
{
namespace Localisation
{
SpeedEstimator::SpeedEstimator() : bind(nullptr), speed(0, 0), speed_quality(0)
{
  initBinding();
}

void SpeedEstimator::update(const rhoban_utils::TimeStamp& ts, const Eigen::Vector2d& pos)
{
  // Debug message
  if (debug_level > 0)
  {
    logger.log("update: pos:  %f, %f, time: %f [s]", pos.x(), pos.y(), ts.getTimeSec());
  }
  // Insert entry
  positions.push_front(TimedPosition(ts, pos));

  bind->pull();

  cleanOldEntries();

  // Each pair of position gives a speed difference
  updateMeasuredSpeeds();

  // Compute mean using weights
  Eigen::Vector2d totSpeeds(0, 0);
  double totWeight = 0;
  for (unsigned int i = 0; i < measured_speeds.size(); i++)
  {
    const Eigen::Vector2d& speed = measured_speeds[i].first;
    double weight = measured_speeds[i].second;
    totSpeeds = totSpeeds + speed * weight;
    totWeight += weight;
    if (debug_level > 1)
    {
      logger.log("measured_speeds[%d] : {speed: (%f,%f), weight: %f }", i, speed.x(), speed.y(), weight);
    }
  }

  if (totWeight > 0)
  {
    Eigen::Vector2d avgSpeed = totSpeeds / totWeight;

    // Compute sum of squared errors weighted
    double norm_var = 0;
    for (unsigned int i = 0; i < measured_speeds.size(); i++)
    {
      const Eigen::Vector2d& speed = measured_speeds[i].first;
      double weight = measured_speeds[i].second;
      //
      double squared_error = (speed - avgSpeed).squaredNorm();
      norm_var += squared_error * weight;
    }
    norm_var /= totWeight;
    // Getting speed deviation
    double speed_dev = std::sqrt(norm_var);

    // Quality of speed depends on speed_dev
    speed_quality = std::max(0.0, 1 - speed_dev / max_speed_dev);

    speed = avgSpeed;
    usable_speed = avgSpeed * speed_quality;

    if (debug_level > 0)
    {
      logger.log("Raw speed    : (%f,%f) ", speed.x(), speed.y());
      logger.log("Usable speed : (%f,%f) ", usable_speed.x(), usable_speed.y());
      logger.log("Quality      : (%f) ", speed_quality);
    }
  }
  else
  {  // totWeight == 0
    speed_quality = 0;
    speed = Eigen::Vector2d(0, 0);
    usable_speed = Eigen::Vector2d(0, 0);
  }

  bind->push();
}

Eigen::Vector2d SpeedEstimator::getSpeed()
{
  return speed;
}

double SpeedEstimator::getQuality()
{
  return speed_quality;
}
Eigen::Vector2d SpeedEstimator::getUsableSpeed()
{
  return usable_speed;
}

void SpeedEstimator::initBinding()
{
  if (bind != nullptr)
  {
    throw std::logic_error("SpeedEstimator already intialized");
  }

  bind = new RhIO::Bind("/ball/speedEstimator");
  bind->bindNew("memory_size", memory_size, RhIO::Bind::PullOnly)
      ->comment("Number of ball positions stored in memory")
      ->defaultValue(15)
      ->minimum(2)
      ->maximum(50);
  bind->bindNew("min_dt", min_dt, RhIO::Bind::PullOnly)
      ->comment("Minimal time difference between to entries to compute their speed [s]")
      ->defaultValue(0.1)
      ->minimum(0.01)
      ->maximum(0.5);
  bind->bindNew("max_dt", max_dt, RhIO::Bind::PullOnly)
      ->comment("Maximal time difference between first and last entry [s]")
      ->defaultValue(0.25)
      ->minimum(0.1)
      ->maximum(1);
  bind->bindNew("disc", disc, RhIO::Bind::PullOnly)
      ->comment("Discount gain used for speeds")
      ->defaultValue(0.9)
      ->minimum(0)
      ->maximum(1);
  bind->bindNew("max_speed_dev", max_speed_dev, RhIO::Bind::PullOnly)
      ->comment("Maximal standard error on speed before usable_speed "
                "reaches 0 [m/s]")
      ->defaultValue(2)
      ->minimum(0)
      ->maximum(2);
  bind->bindNew("max_speed", max_speed, RhIO::Bind::PullOnly)
      ->comment("Maximal theoric speed of the ball [m/s], higher values are "
                "considered as two different ball candidates and are simply ignored")
      ->defaultValue(3);
  bind->bindNew("debug_level", debug_level, RhIO::Bind::PullOnly)
      ->comment("Verbosity of output")
      ->defaultValue(0)
      ->minimum(0)
      ->maximum(2);
  // TODO: eventually output speeds and qSpeed to RhIO

  bind->pull();  // Ensures proper initialization of all RhIO PullOnly variable
}

void SpeedEstimator::cleanOldEntries()
{
  while (positions.size() > (unsigned int)memory_size)
  {
    positions.pop_back();
  }

  // Ensure there is never a difference higher than max_dt
  while (positions.size() > 0 && diffSec(positions.back().first, positions.front().first) > max_dt)
  {
    positions.pop_back();
  }
}

void SpeedEstimator::updateMeasuredSpeeds()
{
  measured_speeds.clear();
  auto newP = positions.cbegin();
  auto oldP = positions.cbegin();
  double age_weight = 1.0;
  while (true)
  {
    double dt = diffSec(oldP->first, newP->first);

    if (dt >= min_dt)
    {
      const Eigen::Vector2d& src = oldP->second;
      const Eigen::Vector2d& dst = newP->second;
      Eigen::Vector2d speed = (dst - src) / dt;
      // The weight of each pair of positions is influenced by age
      // (more recent values are more important)
      double weight = age_weight;
      age_weight *= disc;
      if (speed.norm() > max_speed)
      {
        if (debug_level > 0)
        {
          logger.warning("ignoring improbable speed: (%f,%f)", speed.x(), speed.y());
        }
      }
      else
      {
        measured_speeds.push_back(WeightedSpeed(speed, weight));
      }
      // Reduce dt by taking an older entry as 'new'
      newP++;
    }
    else
    {
      // Computing speed is forbidden, thus trying to increase dt by taking an older entry
      if (oldP == positions.cend())
      {
        break;  // Exit loop if we reach the last entry
      }
      else
      {
        oldP++;
      }
    }
  }
}

}  // namespace Localisation
}  // namespace Vision
