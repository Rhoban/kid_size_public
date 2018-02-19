#include "SpeedEstimator.hpp"
#include "rhoban_utils/timing/time_stamp.h"
#include "rhoban_utils/logging/logger.h"

#include <cmath>

using namespace rhoban_geometry;

using ::rhoban_utils::TimeStamp;
using namespace std;

static rhoban_utils::Logger logger("SpeedEstimator");

namespace Vision {
namespace Localisation {

SpeedEstimator::SpeedEstimator()
  : memorySize(5), disc(0.9),speed(0, 0), qSpeed(0, 0) {}

void SpeedEstimator::update(const rhoban_utils::TimeStamp & ts,
                            const Point &p, double quality) {
  logger.log("update: pos:  %f, %f, time: %f [s], quality: %f",
             p.x, p.y, ts.getTimeSec(), quality);
  // Insert entry
  positions.push_front(TimedPosition(ts, p));
  qualities.push_front(quality);
  if (positions.size() > memorySize) {
    positions.pop_back();
    qualities.pop_back();
  }

  // Wait to fill the memorySize until analyzing
  if (positions.size() < memorySize) {
    return;
  }

  // Each pair of position gives a speed difference
  std::vector<Point> speeds;
  auto newP = positions.begin();
  auto oldP = positions.begin();
  oldP++;
  while (oldP != positions.end()) {
    const Point & src = oldP->second;
    const Point & dst = newP->second;
    Point diff = dst - src;
    double dt = diffSec(oldP->first, newP->first);
    if (dt != 0) { // Not calculating speeds based on no dt
      logger.log("oldPos: (%f,%f), newPos: (%f,%f), diff: (%f,%f), dt: %f",
                 src.x, src.y, dst.x, dst.y, diff.x, diff.y, dt);
                 
      speeds.push_back(diff / dt);
    }
    oldP++;
    newP++;
  }
  // The weight of each pair of positions is influenced by both:
  // - age (more recent values are more important)
  // - quality product
  std::vector<double> weights;
  auto newQ = qualities.begin();
  auto oldQ = qualities.begin();
  oldQ++;
  double ageWeight = 1.0;
  double maxWeight = 0;
  while (oldQ != qualities.end()) {
    weights.push_back(ageWeight * (*oldQ) * (*newQ));
    maxWeight += ageWeight;
    // Updating pointers and weight
    ageWeight *= disc;
    oldQ++;
    newQ++;
  }

  // Compute mean using weights
  Point totSpeeds(0, 0);
  double totWeight = 0;
  for (unsigned int i = 0; i < speeds.size(); i++) {
    totSpeeds = totSpeeds + speeds[i] * weights[i];
    totWeight += weights[i];
  }
  if (totWeight > 0) {
    Point avgSpeed = totSpeeds / totWeight;

    // Compute stdDev using weights
    Point totDev;
    for (unsigned int i = 0; i < speeds.size(); i++) {
      totDev.x += fabs(speeds[i].getX() - avgSpeed.getX());
      totDev.y += fabs(speeds[i].getY() - avgSpeed.getY());
    }
    Point avgDev = totDev / totWeight;

    // Compute quality according to calculated values
    // Quality of estimation depends on:
    // - Coherency of received values
    // - Qualities of positions seen
    Point coherenceScore;
    double flatTol = 0.25;
    coherenceScore.x = (1 - avgDev.getX() / (fabs(avgSpeed.getX() + flatTol)));
    coherenceScore.y = (1 - avgDev.getY() / (fabs(avgSpeed.getY() + flatTol)));
    if (coherenceScore.x < 0)
      coherenceScore.x = 0;
    if (coherenceScore.y < 0)
      coherenceScore.y = 0;
    double qualityScore = totWeight / maxWeight;

    qSpeed = coherenceScore * qualityScore;
    speed = avgSpeed;
  } else { // totWeight <= 0
    qSpeed = Point(0, 0);
    speed = Point(0, 0);
  }
}

Point SpeedEstimator::getSpeed() { return speed; }
double SpeedEstimator::getQuality() { return qSpeed.x * qSpeed.y; }
Point SpeedEstimator::getQualities() { return qSpeed; }
Point SpeedEstimator::getUsableSpeed() {
  return Point(speed.x * qSpeed.x, speed.y * qSpeed.y);
}
}
}
