#pragma once

#include "FieldPosition.hpp"

#include "rhoban_unsorted/particle_filter/particle_filter.h"

#include "RhIO.hpp"

#include <utility>
#include <vector>
#include <map>
#include <mutex>

namespace Vision
{
namespace Localisation
{
class FieldDistribution : public rhoban_unsorted::ParticleFilter<FieldPosition>
{
public:
  struct Distribution
  {
    // first is value, second covmat
    std::pair<rhoban_geometry::Point, double> position;
    float angle;
    int nbParticles;
  };
  std::vector<FieldDistribution::Distribution> updateRepresentativeParticleEM(cv::Mat p,
                                                                              std::vector<rhoban_utils::Angle> a,
                                                                              int size);

protected:
  void EMTrainedLabels(int nbCluster);
  void getMostRecurrentLabel();
  std::pair<rhoban_geometry::Point, double> getPosition(int label);
  float getAngle(int label);
  float getVariance(int label);
  /* return the ration newvariance/oldvariance*/
  float varianceImprovement();

  // bool compareDistribution(const FieldDistribution::Distribution& a, const FieldDistribution::Distribution& b);

  void positionEM();
  void getReccurency();

public:
  FieldDistribution();

  cv::Mat positions;
  std::vector<rhoban_utils::Angle> angles;
  cv::Mat labels;
  cv::Mat oldLabels;

  std::map<int, int> labelMax;
  std::map<int, int> oldLabelMax;
  std::vector<Distribution> result;

  std::pair<int, int> labelMaximum;

  int nbParticles;
  double epsilon;
  float lastVar;
};

}  // namespace Localisation

}  // namespace Vision
