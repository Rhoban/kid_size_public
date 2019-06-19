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
typedef std::vector<rhoban_geometry::Point> PositionClusters;
typedef std::vector<rhoban_utils::Angle> AngleClusters;

std::vector<hl_communication::WeightedPose> updateEM(const cv::Mat& positions, const cv::Mat& angles, int max_clusters);

void exportToProto(const PositionClusters& pos_clusters, const AngleClusters& angle_clusters,
                   hl_communication::WeightedPose* self_in_field, int nbParticles);

void EMTrainedLabels(const cv::Mat& pos, int nb_clusters, cv::Mat* labels, cv::Mat* probs, cv::Mat* log_likelihood,
                     int nb_iterations, double epsilon);

void getClusters(const cv::Mat& positions, const cv::Mat& angles, const cv::Mat& labels, int nbParticles,
                 std::map<int, PositionClusters>* pos_clusters, std::map<int, AngleClusters>* angle_clusters);

double getVariance(const std::map<int, PositionClusters>& clusters);
double getVariance(const PositionClusters& cluster);
double getVariance(const std::map<int, AngleClusters>& clusters);
double getVariance(const AngleClusters& cluster);

}  // namespace Localisation

}  // namespace Vision
