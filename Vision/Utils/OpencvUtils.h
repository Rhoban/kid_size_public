#ifndef VISION_UTILS_OPENCVUTILS_H
#define VISION_UTILS_OPENCVUTILS_H

#include <opencv2/core/core.hpp>

#include <Eigen/Core>

namespace Vision
{
namespace Utils
{
/**
 * Compute image histogram using one color channel
 * img: the image from which the histogram will be computed
 * canalNo: the number of the canal to use
 * hist: The Matrix where the histogram will be placed,
 * it doesn't need to be initialized
 * The histogram will be calculated from valMin to valMax
 * by a step of divSize
 */
void monocanalHist(const cv::Mat& img, unsigned int canalNo, cv::MatND& hist, int valMin, int valMax,
                   unsigned int nbDivisions);

/**
 * Find min and max value with their index
 * in a one dimensional histogram
 * If any value is not needed, nullptr can be given as pointer
 */
void minMaxHist(const cv::MatND& hist, float* minVal = nullptr, float* maxVal = nullptr, int* minIndex = nullptr,
                int* maxIndex = nullptr);

/**
 * hist : a filled histogram in 1 dimension
 * histImg : doesn't need to be initialized
 * scale : width of a column
 */
void drawHist(const cv::MatND& hist, cv::Mat& img, unsigned int scale = 2, unsigned int height = 640);

/**
 * Return a string describing the type of an opencv matrix
 */
std::string cvtype2str(int type);

/**
 * Describe the properties of an opencv image in a string
 */
std::string describeImage(const cv::Mat& m);

/**
 * All the used... functions supports only CV_8U type
 * These functions also supports invalid range, by selecting only the valid
 * part of a range, e.g. :
 * Center = [2,2] dRow = 10 , dCol = 2 -> [0,12] x [0,4]
 */

/**
 * Return a vector containing all the points where value is not zero.
 */
std::vector<cv::Point> usedPoints(const cv::Mat& m);

/**
 * Return a vector containing the used points in the neighborhood.
 * Points examinated are in :
 * row : [center.row - dRow, center.row + dRow]
 * col : [center.col - dCol, center.col + dCol]
 */
std::vector<cv::Point> usedNeighborhood(const cv::Mat& m, const cv::Point& center, int dRow, int dCol);

/**
 * Return a vector containing the used points in the neighborhood.
 * Points examinated are in :
 * row : [center.row - dRow1, center.row + dRow1]
 * col : [center.col - dCol1, center.col + dCol1]
 */
std::vector<cv::Point> usedNeighborhood(const cv::Mat& m, const cv::Point& center, int dRow1, int dRow2, int dCol1,
                                        int dCol2);

/// Return the list of the points inside of the given convex hull in an image
/// Throw a cv::Exception if input is invalid (less than 3 points for example)
/// WARNING: is not optimized at all
std::vector<cv::Point> getPointsInConvexHull(const std::vector<cv::Point>& points, const cv::Mat& img);

cv::Point3f eigenToCV(Eigen::Vector3d& vector);

}  // namespace Utils
}  // namespace Vision

#endif
