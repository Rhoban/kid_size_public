#include "Hough.hpp"

#include <opencv2/imgproc/imgproc.hpp>

#include "Utils/Drawing.hpp"
#include "Utils/Interface.h"
#include "Utils/OpencvUtils.h"
#include "rhoban_utils/timing/benchmark.h"
#include <opencv2/highgui/highgui.hpp>

using rhoban_utils::Benchmark;

using namespace rhoban_geometry;

#define DEBUG_ON false

namespace Vision
{
std::vector<PLCluster> linesClustersRHT(std::vector<cv::Point2i> edges, int nbIterations, float minDist, float rhoTol,
                                        float thetaTol)
{
  std::vector<PLCluster> clusters;
  // Step 1: Sampling lines
  std::vector<ParametricLine> randomLines;
  Benchmark::open("Sampling lines");
  std::random_device rd;   // only used once to initialise (seed) engine
  std::mt19937 rng(rd());  // random-number engine used (Mersenne-Twister in this case)
  std::uniform_int_distribution<int> uni(0, (int)edges.size());  // guaranteed unbiased
  // auto random_integer = uni(rng);
  // std::cout<<"RAND "<<uni(rng)<<" "<<rand()<<std::endl;
  for (int i = 0; i < nbIterations; i++)
  {
    if (edges.size() == 0)
    {
      continue;
    }
    Point p1 = cv2rg(edges[uni(rng)]);
    Point p2 = cv2rg(edges[uni(rng)]);
    if (p1.getDist(p2) > minDist)
    {
      randomLines.push_back(ParametricLine(p1, p2));
    }
  }
  Benchmark::close("Sampling lines");
  // Step 2: Creating clusters;
  Benchmark::open("Adding lines to clusters");
  for (const ParametricLine& l : randomLines)
  {
    addToClusters(l, clusters, rhoTol, thetaTol, 10);
  }
  Benchmark::close("Adding lines to clusters");
  return clusters;
}

std::vector<PLCluster> linesClustersRHT(std::vector<std::vector<cv::Point2i>> edgesSet, int nbIterations, float minDist,
                                        float rhoTol, float thetaTol)
{
  std::vector<PLCluster> clusters;
  // Step 1: Sampling lines
  std::vector<ParametricLine> randomLines;
  float totalSize = 0;
  for (const auto& edge : edgesSet)
  {
    totalSize += edge.size();
  }
  if (totalSize == 0)
  {
    return clusters;
  }

  Benchmark::open("Sampling lines");
  std::random_device rd;                                // only used once to initialise (seed) engine
  std::mt19937 rng(rd());                               // random-number engine used (Mersenne-Twister in this case)
  std::uniform_int_distribution<int> uni(0, RAND_MAX);  // guaranteed unbiased
  // std::cout<<"RAND "<<uni(rng)<<" "<<rand()<<std::endl;
  for (const auto& edge : edgesSet)
  {
    unsigned int setIterations = nbIterations * edge.size() / totalSize;
    for (unsigned int i = 0; i < setIterations; i++)
    {
      Point p1 = cv2rg(edge[rand() % edge.size()]);
      Point p2 = cv2rg(edge[rand() % edge.size()]);
      if (p1.getDist(p2) > minDist)
      {
        randomLines.push_back(ParametricLine(p1, p2));
      }
    }
  }
  Benchmark::close("Sampling lines");
  // Step 2: Creating clusters;
  Benchmark::open("Adding lines to clusters");
  for (const ParametricLine& l : randomLines)
  {
    addToClusters(l, clusters, rhoTol, thetaTol, 10);
  }
  Benchmark::close("Adding lines to clusters");
  return clusters;
}

std::vector<cv::Point2i> eraseLine(const std::vector<cv::Point2i>& edges, const ParametricLine& line, float cleaningTol)
{
  std::vector<cv::Point2i> newEdge;
  for (const auto& p : edges)
  {
    if (line.distanceToPoint(cv2rg(p)) > cleaningTol)
    {
      newEdge.push_back(p);
    }
  }
  return newEdge;
}

std::vector<std::vector<cv::Point2i>> eraseLine(const std::vector<std::vector<cv::Point2i>>& edgeSet,
                                                const ParametricLine& line, float cleaningTol)
{
  std::vector<std::vector<cv::Point2i>> newEdgeSet;
  for (const auto& edge : edgeSet)
  {
    newEdgeSet.push_back(eraseLine(edge, line, cleaningTol));
  }

  return newEdgeSet;
}

std::vector<ParametricLine> detectLinesRHT(const std::vector<cv::Point>& edges, int nbEpochs, int iterationsByEpoch,
                                           int minEdgeSize, int minScore, float eraseThickness, float minDist,
                                           float rhoTol, float thetaTol, cv::Mat* tagImg)
{
  std::vector<cv::Point> workingEdges = edges;
  std::vector<ParametricLine> result;
  for (int epoch = 0; epoch < nbEpochs; epoch++)
  {
    std::vector<PLCluster> clusters = linesClustersRHT(workingEdges, iterationsByEpoch, minDist, rhoTol, thetaTol);
    ParametricLine bestLine;
    double bestScore = 0;
    for (const auto& c : clusters)
    {
      if (c.size() > bestScore)
      {
        bestScore = c.size();
        bestLine = c.getAverage();
      }
    }
    if (bestScore > minScore)
    {
      result.push_back(bestLine);
      workingEdges = eraseLine(workingEdges, bestLine, eraseThickness);
      // TODO : search or write drawing functions on 8UC1 images
      if (tagImg != NULL)
      {
        Utils::draw(*tagImg, bestLine, cv::Scalar(255, 0, 255), 1);
      }
    }
  }
  return result;
}

std::vector<ParametricLine> detectLinesRHT(const std::vector<std::vector<cv::Point>>& workingEdgeSet, int nbEpochs,
                                           int iterationsByEpoch, int minEdgeSize, int minScore, float eraseThickness,
                                           float minDist, float rhoTol, float thetaTol, cv::Mat* tagImg)
{
  std::vector<std::vector<cv::Point>> workingEdges = workingEdgeSet;
  std::vector<ParametricLine> result;
  for (int epoch = 0; epoch < nbEpochs; epoch++)
  {
    std::vector<PLCluster> clusters = linesClustersRHT(workingEdges, iterationsByEpoch, minDist, rhoTol, thetaTol);

    ParametricLine bestLine;
    double bestScore = 0;
    for (const auto& c : clusters)
    {
      if (c.size() > bestScore)
      {
        bestScore = c.size();
        bestLine = c.getAverage();
      }
    }
    if (bestScore > minScore)
    {
      result.push_back(bestLine);

      workingEdges = eraseLine(workingEdges, bestLine, eraseThickness);

      // TODO : search or write drawing functions on 8UC1 images
      if (tagImg != NULL)
      {
        Utils::draw(*tagImg, bestLine, cv::Scalar(255, 0, 255), 1);
      }
    }
  }
  return result;
}

std::vector<ParametricLine> detectLinesRHT(const cv::Mat& img, int nbEpochs, int iterationsByEpoch, int minEdgeSize,
                                           int minScore, int eraseThickness, float minDist, float rhoTol,
                                           float thetaTol, cv::Mat* tagImg)
{
  std::vector<cv::Point> edges = Vision::Utils::usedPoints(img);
  return detectLinesRHT(edges, nbEpochs, iterationsByEpoch, minEdgeSize, minScore, eraseThickness, minDist, rhoTol,
                        thetaTol, tagImg);
}

/**
 * /!\workingImg must be a 1D unsigned char mat.
 * The method detects lines contained in listOfEdges and erases them in
 * workingImg
 */
void eraseLinesRHT(std::vector<std::vector<cv::Point2i>>& listOfEdges, int nbIterations, int minEdgeSize, int minScore,
                   int eraseThickness, float minDist, float rhoTol, float thetaTol, cv::Mat& workingImg)
{
  std::vector<PLCluster> clusters;
  std::vector<ParametricLine> result;
  int totalEdgesSize = 0;
  for (auto contour : listOfEdges)
  {
    // Getting the sum of the edges sizes
    if ((int)contour.size() < minEdgeSize)
    {
      // Neglecting the contours that are too short
      continue;
    }
    else
    {
      totalEdgesSize = totalEdgesSize + contour.size();
    }
  }
  if (totalEdgesSize == 0)
  {
    return;
  }

  for (auto contour : listOfEdges)
  {
    if ((int)contour.size() < minEdgeSize)
    {
      // Neglecting the contours that are too short
      continue;
    }

    std::vector<ParametricLine> randomLines;
    // Computing the candidate Lines and adding them into clusters
    Benchmark::open("Sampling lines");
    std::random_device rd;                                // only used once to initialise (seed) engine
    std::mt19937 rng(rd());                               // random-number engine used (Mersenne-Twister in this case)
    std::uniform_int_distribution<int> uni(0, RAND_MAX);  // guaranteed unbiased
    // std::cout<<"RAND "<<uni(rng)<<" "<<rand()<<std::endl;
    // Each contour will have a portion of the total nbIterations, provided that
    // it is big enough
    for (int i = 0; i < nbIterations; i++)
    {
      Point p1 = cv2rg(contour[rand() % contour.size()]);
      Point p2 = cv2rg(contour[rand() % contour.size()]);
      if (p1.getDist(p2) > minDist)
      {
        randomLines.push_back(ParametricLine(p1, p2));
      }
    }
    Benchmark::close("Sampling lines");
    Benchmark::open("Adding lines to clusters");
    for (const ParametricLine& l : randomLines)
    {
      // The last param is the max number of different clusters.
      addToClusters(l, clusters, rhoTol, thetaTol, 1024);
    }
    Benchmark::close("Adding lines to clusters");
  }

  Benchmark::open("Choosing and deletingg lines");
  for (const auto& c : clusters)
  {
    // Keping the candidates that are good enough
    if ((int)c.size() >= minScore)
    {
      ParametricLine validLine = c.getAverage();
      // double score = c.size();
      result.push_back(validLine);
    }
  }
  // Creating a white image
  cv::Mat tmpImg(workingImg.size(), 0, cv::Scalar(255));
  for (auto validLine : result)
  {
    // Drawing the lines in black
    Utils::draw(tmpImg, validLine, cv::Scalar(0, 0, 0), eraseThickness);
  }

  // Deleting the lines in the original image
  workingImg = workingImg & tmpImg;
  Benchmark::close("Choosing and deletingg lines");
#if DEBUG_ON
  cv::namedWindow("HoughDebug", CV_WINDOW_AUTOSIZE);
  cv::imshow("HoughDebug", tmpImg);

  cv::namedWindow("workingImg", CV_WINDOW_AUTOSIZE);
  cv::imshow("workingImg", workingImg);
#endif

  return;
}

std::vector<Circle> generateCircleCandidates(const std::vector<cv::Point>& edge, int max_candidates, double min_dist,
                                             std::default_random_engine* engine)
{
  std::vector<Circle> candidates;
  std::uniform_int_distribution<int> edge_distrib(0, edge.size() - 1);
  for (int e = 0; e < max_candidates; e++)
  {
    cv::Point p[3];
    bool too_close = false;
    for (int i = 0; i < 3; i++)
    {
      int index = edge_distrib(*engine);
      p[i] = edge[index];
      for (int j = i - 1; j >= 0; j--)
      {
        double dist = cv::norm(p[i] - p[j]);
        if (dist < min_dist)
        {
          too_close = true;
        }
      }
    }
    // Points of this generation are not used
    if (too_close)
      continue;

    try
    {
      Circle c = Circle::circleFromPoints(cv2rg(p[0]), cv2rg(p[1]), cv2rg(p[2]));
      candidates.push_back(c);
    }
    // If points are not collinear, ignore situation
    catch (const ImpossibleCircleException& exc)
    {
    }
  }
  return candidates;
}

std::vector<Circle> generateCircleCandidates(const std::vector<std::vector<cv::Point>>& edges, int total_candidates,
                                             size_t min_edge_size, double min_dist, std::default_random_engine* engine)
{
  // First, filter edges which are too small and count number of pixels
  std::vector<size_t> valid_edges_id;
  int valid_pixels = 0;
  for (size_t edge_id = 0; edge_id < edges.size(); edge_id++)
  {
    size_t edge_size = edges[edge_id].size();
    // Skip small edges
    if (edge_size < min_edge_size)
      continue;
    // Add large edges
    valid_edges_id.push_back(edge_id);
    valid_pixels += edge_size;
  }
  // Now compute all circle
  std::vector<Circle> result;
  for (size_t edge_id : valid_edges_id)
  {
    const std::vector<cv::Point>& edge = edges[edge_id];
    double edge_ratio = edge.size() / (double)valid_pixels;
    int edge_max_candidates = std::floor(edge_ratio * total_candidates);
    std::vector<Circle> edge_candidates = generateCircleCandidates(edge, edge_max_candidates, min_dist, engine);
    result.insert(result.end(), edge_candidates.begin(), edge_candidates.end());
  }
  return result;
}
}  // namespace Vision
