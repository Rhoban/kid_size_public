#pragma once

#include "rhoban_geometry/circle.h"
#include "rhoban_geometry/circle_cluster.h"
#include "rhoban_geometry/parametric_line.h"
#include "rhoban_geometry/parametric_line_cluster.h"

#include <opencv2/core/core.hpp>

#include <random>

// RHT: Randomized Hough Transform

namespace Vision {

/**
 * Calculate a vector of ParametricLines clusters using RHT on the given edges
 */
std::vector<rhoban_geometry::PLCluster>
linesClustersRHT(std::vector<cv::Point2i> edges,
                 int nbIterations, float minDist,
                 float rhoTol, float thetaTol);

/**
 * Variant of linesClustersRHT, a set of edges is given and the lines are always
 * sampled on two
 * point belonging to the same set.
 * The real number of iterations might be lower due to approximations
 */
std::vector<rhoban_geometry::PLCluster>
linesClustersRHT(std::vector<std::vector<cv::Point2i>> edgesSets,
                 int nbIterations, float minDist, float rhoTol, float thetaTol);

/**
 * Return a new set of edges, by removing all the pixels which are closer to
 * the line than cleaningTol
 */
std::vector<cv::Point2i> eraseLine(const std::vector<cv::Point2i> &edges,
                                   const rhoban_geometry::ParametricLine &line,
                                   float cleaningTol);
std::vector<std::vector<cv::Point2i>>
eraseLine(const std::vector<std::vector<cv::Point2i>> &edges,
          const rhoban_geometry::ParametricLine &line, float cleaningTol);

/**
 * Detect lines by RHT from the given blobs and parameters
 */
std::vector<rhoban_geometry::ParametricLine>
detectLinesRHT(const std::vector<cv::Point> &edges,
               int nbEpochs, int iterationsByEpoch,
               int minEdgeSize, int minScore,
               float eraseThickness, float minDist,
               float rhoTol, float thetaTol,
               cv::Mat *tagImg = NULL);

std::vector<rhoban_geometry::ParametricLine>
detectLinesRHT(const std::vector<std::vector<cv::Point>> &edges, int nbEpochs,
               int iterationsByEpoch, int minEdgeSize, int minScore,
               float eraseThickness, float minDist, float rhoTol,
               float thetaTol, cv::Mat *tagImg = NULL);

std::vector<rhoban_geometry::ParametricLine>
detectLinesRHT(const cv::Mat &img, int nbEpochs, int iterationsByEpoch,
               int minEdgeSize, int minScore, int eraseThickness, float minDist,
               float rhoTol, float thetaTol, cv::Mat *tagImg = NULL);

void eraseLinesRHT(std::vector<std::vector<cv::Point2i>> &listOfEdges,
                   int iterationsByEpoch, int minEdgeSize, int minScore,
                   int eraseThickness, float minDist, float rhoTol,
                   float thetaTol, cv::Mat &workingImg);

/// Repeat 'max_candidates' time:
/// - draw 3 random points
/// - if distance between points is higher than min_dist
///   - Create a circle and add it to a circle vector
/// - return the vector circle containing valid candidates
std::vector<rhoban_geometry::Circle>
generateCircleCandidates(const std::vector<cv::Point> &edge, int max_candidates,
                         double min_dist, std::default_random_engine *engine);

/// Generate a set of candidates for the circles
/// - number of candidates is distributed among edges depending on their size
std::vector<rhoban_geometry::Circle>
generateCircleCandidates(const std::vector<std::vector<cv::Point>> &edges,
                         int total_candidates, size_t min_edge_size,
                         double min_dist, std::default_random_engine *engine);
}
