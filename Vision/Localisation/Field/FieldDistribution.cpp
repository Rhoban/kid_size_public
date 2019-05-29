#include <iostream>

#include "FieldDistribution.hpp"

#include "rhoban_utils/logging/logger.h"

#include <hl_monitoring/top_view_drawer.h>
#include <hl_communication/perception.pb.h>
#include <robocup_referee/constants.h>
#include "rhoban_random/multivariate_gaussian.h"

#include <opencv2/imgproc.hpp>
#include <opencv2/ml.hpp>
#include <vector>
#include <algorithm>
#include <map>
#include <sstream>
#include <cmath>

static rhoban_utils::Logger logger("FieldDistribution");

using namespace std;
using namespace rhoban_geometry;
using namespace rhoban_utils;
using namespace rhoban_random;
using namespace rhoban_unsorted;
using namespace robocup_referee;
using namespace hl_monitoring;
using namespace hl_communication;

namespace Vision
{
namespace Localisation
{
FieldDistribution::FieldDistribution() : ParticleFilter()
{
}

hl_communication::WeightedPose* FieldDistribution::distributionToProto(FieldDistribution::Distribution d)
{
  Eigen::MatrixXd m = d.position.second;
  std::cout << "m recieved " << std::endl;

  hl_communication::WeightedPose* self_in_field;
  std::cout << " init ok " << std::endl;
  self_in_field->set_probability(d.probability);
  std::cout << "pro sent " << std::endl;
  self_in_field->mutable_pose()->mutable_position()->set_x(d.position.first.getX());
  self_in_field->mutable_pose()->mutable_position()->set_y(d.position.first.getY());
  std::cout << "pos sent " << std::endl;
  self_in_field->mutable_pose()->mutable_dir()->set_mean(deg2rad(d.angle.first));
  std::cout << "angle sent " << std::endl;
  self_in_field->mutable_pose()->mutable_position()->add_uncertainty(m(0, 0));
  self_in_field->mutable_pose()->mutable_position()->add_uncertainty(m(1, 0));
  self_in_field->mutable_pose()->mutable_position()->add_uncertainty(m(1, 1));
  std::cout << "uncertainty sent " << std::endl;
  return self_in_field;
}

void FieldDistribution::EMTrainedLabels(int nbCluster)
{
  cv::Mat probs;
  cv::Mat logLikelihoods;
  cv::Mat samples = positions.reshape(1, 0);

  // training
  static cv::Ptr<cv::ml::EM> em_model = cv::ml::EM::create();

  em_model->setClustersNumber(nbCluster);
  em_model->setCovarianceMatrixType(cv::ml::EM::COV_MAT_DIAGONAL);
  em_model->setTermCriteria(cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 300, 0.1));
  em_model->trainEM(samples, probs, labels, logLikelihoods);
}

void FieldDistribution::getMostRecurrentLabel()
{
  labelMax.clear();

  // counting number of repetition of each label
  std::map<int, int> clusters;
  for (int i = 0; i < nbParticles; i++)
  {
    if (clusters.find(labels.at<int>(i)) == clusters.end())
      clusters[labels.at<int>(i)] = 1;
    else
      clusters[labels.at<int>(i)]++;
  }

  // getting most recurrent labels
  for (auto it = clusters.begin(); it != clusters.end(); it++)
  {
    // if there is enough repetition of it
    if (clusters[it->first] > (int)epsilon)
      labelMax[it->first] = clusters[it->first];
  }
}

void FieldDistribution::printLabel()
{
  for (auto it = oldLabelMax.begin(); it != oldLabelMax.end(); it++)
  {
    std::cout << "label n°" << it->first << " quantity : " << labelMax[it->first] << std::endl;
    Distribution newd;
    newd.position = getPosition(it->first);
    newd.angle = getAngle(it->first);
    std::cout << "x : " << newd.position.first.x << " y : " << newd.position.first.y << " angle : " << newd.angle.first
              << std::endl;
  }
}

std::vector<FieldDistribution::Distribution> FieldDistribution::updateEM(cv::Mat p, std::vector<Angle> a, int size)
{
  result.clear();
  labelMax.clear();

  positions = p;
  angles = a;
  nbParticles = size;
  epsilon = 5 / 100 * nbParticles;

  lastVar.first = getVariancePosition(-1);
  lastVar.second = getVarianceDirection(-1);

  std::cout << "first variance for position " << lastVar.first << " and direction " << lastVar.second << std::endl;

  int nbCluster = 2;
  std::pair<double, double> varReduction(0, 0);
  while ((varReduction.first < 0.5 || varReduction.second < 0.5) && nbCluster <= 5)
  {
    oldLabels = labels;
    oldLabelMax = labelMax;
    labelMax.clear();
    EMTrainedLabels(nbCluster);
    getMostRecurrentLabel();
    varReduction = varianceImprovement();
    nbCluster++;
  }

  printLabel();

  if (oldLabelMax.begin() == oldLabelMax.end())
  {
    oldLabels = labels;
    Distribution newd;
    newd.position = getPosition(-1);
    newd.angle = getAngle(-1);
    newd.probability = 1;
    result.push_back(newd);
  }
  else
  {
    for (auto it = oldLabelMax.begin(); it != oldLabelMax.end(); it++)
    {
      Distribution newd;
      newd.position = getPosition(it->first);
      newd.angle = getAngle(it->first);
      newd.probability = labelMax[it->first] / nbParticles;
      result.push_back(newd);
    }

    // sort from larger to smaller label
    std::sort(result.begin(), result.end(),
              [](const FieldDistribution::Distribution& a, const FieldDistribution ::Distribution& b) {
                return a.probability > b.probability;
              });
  }

  return result;
}

std::pair<Point, Eigen::MatrixXd> FieldDistribution::getPosition(int label)
{
  // getting the major group
  std::vector<Eigen::VectorXd> cluster;
  for (int i = 0; i < nbParticles; i++)
  {
    if (oldLabels.at<int>(i) == label || label < 0)
    {
      double x = positions.at<double>(i, 0);
      double y = positions.at<double>(i, 1);
      cluster.push_back((Point(x, y)).toVector());
    }
  }

  Eigen::VectorXd mean;
  Eigen::MatrixXd covMat;

  MultivariateGaussian multivariateGaussian(mean, covMat);

  multivariateGaussian.fit(cluster);

  return std::make_pair(Point(multivariateGaussian.getMean()), multivariateGaussian.getCovariance());
}

std::pair<double, double> FieldDistribution::getAngle(int label)
{
  // getting the major label
  std::vector<Angle> anglelabel;

  for (int i = 0; i < nbParticles; i++)
  {
    if (oldLabels.at<int>(i) == label || label < 0)
    {
      anglelabel.push_back(angles[i]);
    }
  }
  Angle dir = Angle::mean(anglelabel);
  double mean = dir.Angle::getSignedValue();

  double stddev = Angle::stdDev(anglelabel);

  return std::make_pair(mean, stddev);
}

double FieldDistribution::getVariancePosition(int label)
{
  std::vector<rhoban_geometry::Point> cluster;

  for (int i = 0; i < nbParticles; i++)
  {
    if (label < 0 || labels.at<int>(i) == label)
    {
      double x = positions.at<double>(i, 0);
      double y = positions.at<double>(i, 1);
      cluster.push_back(Point(x, y));
    }
  }

  double standardDevation = stdDev(cluster);

  return (standardDevation * standardDevation);
}

double FieldDistribution::getVarianceDirection(int label)
{
  std::vector<Angle> cluster;

  for (int i = 0; i < nbParticles; i++)
  {
    if (label < 0 || labels.at<int>(i) == label)
    {
      cluster.push_back(angles[i]);
    }
  }

  double standardDevation = Angle::stdDev(cluster);

  return (standardDevation * standardDevation);
}

std::pair<double, double> FieldDistribution::varianceImprovement()
{
  std::pair<double, double> variance(0, 0);

  for (auto it = labelMax.begin(); it != labelMax.end(); it++)
  {
    variance.first += getVariancePosition(it->first) * labelMax[it->first];
    variance.second += getVarianceDirection(it->first) * labelMax[it->first];
  }

  variance.first = variance.first / nbParticles;
  variance.second = variance.second / nbParticles;

  std::cout << "updated variance for position " << variance.first << " and direcction" << variance.second << std::endl;

  std::pair<double, double> improvement(variance.first / lastVar.first, variance.second / lastVar.second);
  lastVar = variance;
  return improvement;
}
}  // namespace Localisation

}  // namespace Vision
