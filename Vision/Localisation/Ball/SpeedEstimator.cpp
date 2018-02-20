#include "SpeedEstimator.hpp"
#include "rhoban_utils/timing/time_stamp.h"
#include "rhoban_utils/logging/logger.h"

#include <RhIO.hpp>

#include <cmath>

using namespace rhoban_geometry;

using ::rhoban_utils::TimeStamp;
using namespace std;

static rhoban_utils::Logger logger("SpeedEstimator");

namespace Vision {
namespace Localisation {

SpeedEstimator::SpeedEstimator()
  : bind(nullptr), memory_size(20), min_dt(0.05), max_dt(1),
    disc(0.9), flat_tol(0.25), speed(0, 0), qSpeed(0, 0)
{
  initBinding();
}

void SpeedEstimator::update(const rhoban_utils::TimeStamp & ts,
                            const Point &p, double quality) {
  // Debug message (TODO: make it an option)
  //logger.log("update: pos:  %f, %f, time: %f [s], quality: %f",
  //           p.x, p.y, ts.getTimeSec(), quality);
  // Insert entry
  positions.push_front(TimedPosition(ts, p));
  qualities.push_front(quality);

  bind->pull();

  cleanOldEntries();

  // Each pair of position gives a speed difference
  std::vector<WeightedSpeed> weightedSpeeds = getWeightedSpeeds();
  int nb_entries = weightedSpeeds.size();
  double max_weight = (1 - pow(disc,nb_entries)) / (1 - disc);

  // Compute mean using weights
  Point totSpeeds(0, 0);
  double totWeight = 0;
  for (unsigned int i = 0; i < weightedSpeeds.size(); i++) {
    const Point & speed = weightedSpeeds[i].first;
    double weight = weightedSpeeds[i].second;
    totSpeeds = totSpeeds + speed * weight;
    totWeight += weight;
  }
  if (totWeight > 0) {
    Point avgSpeed = totSpeeds / totWeight;

    // Compute stdDev using weights
    Point totDev(0,0);
    for (unsigned int i = 0; i < weightedSpeeds.size(); i++) {
      const Point & speed = weightedSpeeds[i].first;
      double weight = weightedSpeeds[i].second;
      totDev.x += weight * fabs(speed.getX() - avgSpeed.getX());
      totDev.y += weight * fabs(speed.getY() - avgSpeed.getY());
    }
    Point avgDev = totDev / totWeight;

    // Compute quality according to calculated values
    // Quality of estimation depends on:
    // - Coherency of received values
    // - Qualities of positions seen
    Point coherenceScore;
    // Coherence score is based on 1 - avgError / (speed + flatTol)
    coherenceScore.x = (1 - avgDev.getX() / (fabs(avgSpeed.getX() + flat_tol)));
    coherenceScore.y = (1 - avgDev.getY() / (fabs(avgSpeed.getY() + flat_tol)));
    if (coherenceScore.x < 0)
      coherenceScore.x = 0;
    if (coherenceScore.y < 0)
      coherenceScore.y = 0;
    double qualityScore = totWeight / max_weight;

    qSpeed = coherenceScore * qualityScore;
    speed = avgSpeed;
  } else { // totWeight == 0
    qSpeed = Point(0, 0);
    speed = Point(0, 0);
  }

  bind->push();
}

Point SpeedEstimator::getSpeed() { return speed; }
double SpeedEstimator::getQuality() { return qSpeed.x * qSpeed.y; }
Point SpeedEstimator::getQualities() { return qSpeed; }
Point SpeedEstimator::getUsableSpeed() {
  return Point(speed.x * qSpeed.x, speed.y * qSpeed.y);
}

void SpeedEstimator::initBinding() {
  if (bind != nullptr) {
    throw std::logic_error("SpeedEstimator already intialized");
  }

  bind = new RhIO::Bind("/ball/speedEstimator");
  bind->bindNew("memory_size", memory_size, RhIO::Bind::PullOnly)
    ->comment("Number of ball positions stored in memory")
    ->defaultValue(15)->minimum(2)->maximum(50)->persisted(true);
  bind->bindNew("min_dt", min_dt, RhIO::Bind::PullOnly)
    ->comment("Minimal time difference between to entries to compute their speed [s]")
    ->defaultValue(0.05)->minimum(0.01)->maximum(1)->persisted(true);
  bind->bindNew("max_dt", max_dt, RhIO::Bind::PullOnly)
    ->comment("Maximal time difference between first and last entry [s]")
    ->defaultValue(1)->minimum(0.5)->maximum(10)->persisted(true);
  bind->bindNew("disc", disc, RhIO::Bind::PullOnly)
    ->comment("Discount gain used for speeds")
    ->defaultValue(0.9)->minimum(0)->maximum(1);
  bind->bindNew("flat_tol", flat_tol, RhIO::Bind::PullOnly)
    ->comment("Speed tolerance for coherency evaluation [m/s]")
    ->defaultValue(0.25)->minimum(0)->maximum(2);
  // TODO: eventually output speeds and qSpeed to RhIO
}

void SpeedEstimator::cleanOldEntries() {
  while (positions.size() > (unsigned int)memory_size) {
    positions.pop_back();
    qualities.pop_back();
  }

  // Ensure there is never a difference higher than max_dt
  while (positions.size() > 0 &&
         diffSec(positions.back().first, positions.front().first) > max_dt) {
    positions.pop_back();
    qualities.pop_back();
  }
}

std::vector<SpeedEstimator::WeightedSpeed> SpeedEstimator::getWeightedSpeeds() const {
  std::vector<WeightedSpeed> speeds;
  auto newP = positions.cbegin();
  auto newQ = qualities.cbegin();
  auto oldP = positions.cbegin();
  auto oldQ = qualities.cbegin();
  double age_weight = 1.0;
  while (true) {
    double dt = diffSec(oldP->first, newP->first);

    if (dt >= min_dt) {
      const Point & src = oldP->second;
      const Point & dst = newP->second;
      Point speed = (dst - src) / dt;
      // The weight of each pair of positions is influenced by both:
      // - age (more recent values are more important)
      // - quality product
      double weight = age_weight * (*oldQ) * (*newQ);
      age_weight *= disc;
      speeds.push_back(WeightedSpeed(speed, weight));
      // Reduce dt by taking an older entry as 'new'
      newP++;
      newQ++;
    } else {
      // Computing speed is forbidden, thus trying to increase dt by taking an older entry
      if (oldP == positions.cend()) {
        break;// Exit loop if we reach the last entry
      } else {
        oldP++;
        oldQ++;
      }
    }
  }
  return speeds;
}

}
}
