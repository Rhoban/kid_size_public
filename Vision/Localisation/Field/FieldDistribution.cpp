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
void exportToProto(const PositionClusters& pos_clusters, const AngleClusters& angle_clusters,
                   hl_communication::WeightedPose* self_in_field, int nbParticles)
{
  self_in_field->set_probability((float)pos_clusters.size() / nbParticles);

  Eigen::VectorXd mean;
  Eigen::MatrixXd covMat;

  MultivariateGaussian multivariateGaussian(mean, covMat);
  std::vector<Eigen::VectorXd> cluster;
  for (unsigned int i = 0; i < pos_clusters.size(); i++)
  {
    cluster.push_back(pos_clusters[i].toVector());
  }

  multivariateGaussian.fit(cluster);

  rhoban_geometry::Point p = Point(multivariateGaussian.getMean());
  covMat = multivariateGaussian.getCovariance();

  self_in_field->mutable_pose()->mutable_position()->set_x(p.getX());
  self_in_field->mutable_pose()->mutable_position()->set_y(p.getY());
  self_in_field->mutable_pose()->mutable_position()->add_uncertainty(covMat(0, 0));
  self_in_field->mutable_pose()->mutable_position()->add_uncertainty(covMat(1, 0));
  self_in_field->mutable_pose()->mutable_position()->add_uncertainty(covMat(1, 1));

  Angle dir = Angle::mean(angle_clusters);
  double mean_dir = dir.Angle::getSignedValue();

  double stddev = Angle::stdDev(angle_clusters);

  self_in_field->mutable_pose()->mutable_dir()->set_mean(deg2rad(mean_dir));
  self_in_field->mutable_pose()->mutable_dir()->set_std_dev(deg2rad(stddev));
}

void EMTrainedLabels(const cv::Mat& pos, int nb_clusters, cv::Mat* labels, cv::Mat* probs, cv::Mat* log_likelihood,
                     int nb_iterations = 300, double epsilon = 0.01)

{
  cv::Mat samples = pos;

  // training
  cv::Ptr<cv::ml::EM> em_model = cv::ml::EM::create();

  em_model->setClustersNumber(nb_clusters);
  em_model->setCovarianceMatrixType(cv::ml::EM::COV_MAT_DIAGONAL);
  em_model->setTermCriteria(cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, nb_iterations, epsilon));
  em_model->trainEM(samples, *probs, *labels, *log_likelihood);
}

void getClusters(const cv::Mat& positions, const cv::Mat& angles, const cv::Mat& labels, int nbParticles,
                 std::map<int, PositionClusters>* pos_clusters, std::map<int, AngleClusters>* angle_clusters)
{
  pos_clusters->clear();
  angle_clusters->clear();

  for (int i = 0; i < nbParticles; i++)
  {
    double x = positions.at<double>(i, 0);
    double y = positions.at<double>(i, 1);
    pos_clusters->operator[](labels.at<int>(i)).push_back(Point(x, y));
    angle_clusters->operator[](labels.at<int>(i)).push_back(angles.at<double>(i, 0));
  }
}

std::vector<hl_communication::WeightedPose> updateEM(const cv::Mat& positions, const cv::Mat& angles, int max_clusters)
{
  int nbParticles = positions.rows;

  cv::Mat probs, log_likelihood;
  cv::Mat labels = cv::Mat::zeros(nbParticles, 1, CV_32S);

  std::map<int, PositionClusters> pos_clusters;
  std::map<int, AngleClusters> angle_clusters;

  std::map<int, PositionClusters> old_pos_clusters;
  std::map<int, AngleClusters> old_angle_clusters;

  int nbCluster = 0;
  double newPosVar = 1;
  double newAngleVar = 1;
  double posVar = 1;
  double angleVar = 1;
  double posVarReduction = 1;
  double angleVarReduction = 1;

  while ((posVarReduction > 0.5 || angleVarReduction > 0.5) && nbCluster <= max_clusters)
  {
    old_pos_clusters = pos_clusters;
    old_angle_clusters = angle_clusters;

    nbCluster++;
    EMTrainedLabels(positions, nbCluster, &labels, &probs, &log_likelihood);
    getClusters(positions, angles, labels, nbParticles, &pos_clusters, &angle_clusters);
    posVar = newPosVar;
    angleVar = newAngleVar;
    newPosVar = getVariance(pos_clusters);
    newAngleVar = getVariance(angle_clusters);
    if (nbCluster > 1)
    {
      posVarReduction = 1 - (newPosVar / posVar);
      angleVarReduction = 1 - (newAngleVar / angleVar);
    }
  }

  std::vector<hl_communication::WeightedPose> clusters;

  for (int i = 0; i < nbCluster - 1; i++)
  {
    if (old_pos_clusters[i].size() != old_angle_clusters[i].size())
      throw std::logic_error("different size between angle cluster and position cluster");
    hl_communication::WeightedPose wp;
    exportToProto(old_pos_clusters[i], old_angle_clusters[i], &wp, nbParticles);
    clusters.push_back(wp);
  }

  if (nbCluster > 2)
  {
    // sort from larger to smaller label
    std::sort(clusters.begin(), clusters.end(),
              [](const hl_communication::WeightedPose& a, const hl_communication::WeightedPose& b) {
                return a.probability() > b.probability();
              });
  }

  return clusters;
}

double getVariance(const std::map<int, PositionClusters>& clusters)
{
  double variance = 0;
  int nbParticles = 0;

  for (const auto& entry : clusters)
  {
    variance += getVariance(entry.second) * entry.second.size();
    nbParticles += entry.second.size();
  }
  return variance / nbParticles;
}
double getVariance(const PositionClusters& cluster)
{
  return stdDev(cluster);
}

double getVariance(const std::map<int, AngleClusters>& clusters)
{
  double variance = 0;
  int nbParticles = 0;

  for (const auto& entry : clusters)
  {
    variance += getVariance(entry.second) * entry.second.size();
    nbParticles += entry.second.size();
  }
  return variance / nbParticles;
}

double getVariance(const AngleClusters& cluster)
{
  return Angle::stdDev(cluster);
}

}  // namespace Localisation

}  // namespace Vision
