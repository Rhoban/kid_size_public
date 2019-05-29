#pragma once

#include "FieldPosition.hpp"

#include "rhoban_unsorted/particle_filter/particle_filter.h"
#include <hl_communication/perception.pb.h>

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
    // first is value, second covMat
    std::pair<rhoban_geometry::Point, Eigen::MatrixXd> position;
    // first is value in degres, second stddev
    std::pair<double, double> angle;
    double probability;
  };
  std::vector<FieldDistribution::Distribution> updateEM(cv::Mat p, std::vector<rhoban_utils::Angle> a, int size);

  hl_communication::WeightedPose* distributionToProto(FieldDistribution::Distribution d);

protected:
  void EMTrainedLabels(int nbCluster);
  void getMostRecurrentLabel();
  std::pair<rhoban_geometry::Point, Eigen::MatrixXd> getPosition(int label);
  std::pair<double, double> getAngle(int label);
  double getVariancePosition(int label);
  double getVarianceDirection(int label);
  /* return the ratio newvariance/oldvariance*/
  std::pair<double, double> varianceImprovement();

  void printLabel();

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
  std::pair<double, double> lastVar;
};

}  // namespace Localisation

}  // namespace Vision
