#include <iostream>

#include "FieldDistribution.hpp"

#include "rhoban_utils/logging/logger.h"

#include <hl_monitoring/top_view_drawer.h>
#include <robocup_referee/constants.h>

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
using namespace rhoban_unsorted;
using namespace robocup_referee;
using namespace hl_monitoring;

namespace Vision
{
namespace Localisation
{
FieldDistribution::FieldDistribution() : ParticleFilter()
{
}

void FieldDistribution::EMTrainedLabels(int nbCluster)
{
  // actualiseSamples();
  cv::Mat probs;
  cv::Mat logLikelihoods;
  cv::Mat samples = positions.reshape(1, 0);

  // training
  static cv::Ptr<cv::ml::EM> em_model = cv::ml::EM::create();

  em_model->setClustersNumber(nbCluster);
  em_model->setCovarianceMatrixType(cv::ml::EM::COV_MAT_DIAGONAL);
  em_model->setTermCriteria(cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 300, 0.1));
  if (!samples.empty())
  {
    em_model->trainEM(samples, probs, labels, logLikelihoods);
  }
}

void FieldDistribution::getMostRecurrentLabel()
{
  labelMax.clear();

  // getting number of repetition of each label
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

void FieldDistribution::getReccurency()
{
  getMostRecurrentLabel();

  for (auto it = oldLabelMax.begin(); it != oldLabelMax.end(); it++)
  {
    std::cout << "label n°" << it->first << " quantity : " << labelMax[it->first] << std::endl;
    Distribution newd;
    newd.position = getPosition(it->first);
    newd.angle = getAngle(it->first);
    std::cout << "x : " << newd.position.first.x << " y : " << newd.position.first.y << " angle : " << newd.angle
              << std::endl;
  }
}

std::vector<FieldDistribution::Distribution> FieldDistribution::updateRepresentativeParticleEM(cv::Mat p,
                                                                                               std::vector<Angle> a,
                                                                                               int size)
{
  positions = p;
  angles = a;
  nbParticles = size;
  epsilon = 5 / 100 * nbParticles;

  cv::Scalar mean, stddev;

  lastVar = getVariance(-1);

  std::cout << "first var " << lastVar << std::endl;

  int nbCluster = 2;
  float varReduction = 0;
  while (varReduction < 0.5 && nbCluster <= 5)
  {
    oldLabels = labels;
    oldLabelMax = labelMax;
    EMTrainedLabels(nbCluster);
    getMostRecurrentLabel();
    varReduction = varianceImprovement();
    std::cout << "total var " << varReduction << std::endl;
    nbCluster++;
  }

  getReccurency();

  for (auto it = oldLabelMax.begin(); it != oldLabelMax.end(); it++)
  {
    Distribution newd;
    newd.position = getPosition(it->first);
    newd.angle = getAngle(it->first);
    newd.nbParticles = labelMax[it->first];
    result.push_back(newd);
  }

  std::sort(result.begin(), result.end(),
            [](const FieldDistribution::Distribution& a, const FieldDistribution ::Distribution& b) {
              return a.nbParticles > b.nbParticles;
            });
  /*  std::pair<rhoban_geometry::Point, double> pos = getPosition(labelMaximum.first);
  float dir = getAngle(labelMaximum.first);

  Eigen::VectorXd result(3);

  result << pos.first.getX(), pos.first.getY(), dir;
  std::cout << result;*/
  return result;
}

std::pair<Point, double> FieldDistribution::getPosition(int label)
{
  // getting the major group
  std::vector<rhoban_geometry::Point> cluster;

  for (int i = 0; i < nbParticles; i++)
  {
    if (oldLabels.at<int>(i) == label)
    {
      double x = positions.at<double>(i, 0);
      double y = positions.at<double>(i, 1);
      cluster.push_back(Point(x, y));
    }
  }

  double standardDevation = stdDev(cluster);
  Point mean = average(cluster);

  return std::make_pair(mean, (standardDevation * standardDevation));
}

float FieldDistribution::getAngle(int label)
{
  // getting the major group
  float x(0), y(0);  // computing avg angle
  std::vector<Angle> anglelabel;

  for (int i = 0; i < nbParticles; i++)
  {
    if (oldLabels.at<int>(i) == label)
    {
      anglelabel.push_back(angles[i]);
    }
  }
  Angle dir = Angle::mean(anglelabel);

  float mean = (float)(dir.Angle::getSignedValue());

  return mean;
}

float FieldDistribution::getVariance(int label)
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

  std::cout << " var " << standardDevation << std::endl;
  return (float)(standardDevation * standardDevation);
}

float FieldDistribution::varianceImprovement()
{
  float variance = 0;

  for (auto it = labelMax.begin(); it != labelMax.end(); it++)
  {
    variance += getVariance(it->first) * labelMax[it->first];
  }

  variance = variance / nbParticles;

  if (variance > lastVar)
    return 0.0;
  else
  {
    float improvement = variance / lastVar;
    lastVar = variance;
    return improvement;
  }
}

}  // namespace Localisation

}  // namespace Vision
