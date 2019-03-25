#include "Utils/OpencvUtils.h"

#include <opencv2/imgproc/imgproc.hpp>

namespace Vision
{
namespace Utils
{
void monocanalHist(const cv::Mat& img, unsigned int canalNo, cv::MatND& hist, int valMin, int valMax,
                   unsigned int nbDivisions)
{
  int channels[] = { (int)canalNo };
  int histSize[] = { (int)nbDivisions };
  float valRanges[] = { (float)valMin, (float)valMax };
  const float* ranges[] = { valRanges };
  cv::calcHist(&img, 1, channels, cv::Mat(), hist, 1, histSize, ranges, true, false);
}

void minMaxHist(const cv::MatND& hist, float* minVal, float* maxVal, int* minIndex, int* maxIndex)
{
  float min = hist.at<float>(0);
  float max = hist.at<float>(0);
  int minI = 0;
  int maxI = 0;

  for (int i = 1; i < hist.rows; i++)
  {
    float val = hist.at<float>(i);
    if (val > max)
    {
      max = val;
      maxI = i;
    }
    if (val < min)
    {
      min = val;
      minI = i;
    }
  }

  if (minVal != nullptr)
  {
    *minVal = min;
  }
  if (maxVal != nullptr)
  {
    *maxVal = max;
  }
  if (minIndex != nullptr)
  {
    *minIndex = minI;
  }
  if (maxIndex != nullptr)
  {
    *maxIndex = maxI;
  }
}

void drawHist(const cv::MatND& hist, cv::Mat& img, unsigned int scale, unsigned int height)
{
  float maxVal;
  minMaxHist(hist, nullptr, &maxVal);
  img = cv::Mat::zeros(height, scale * hist.rows, CV_8UC3);

  for (int s = 0; s < hist.rows; s++)
  {
    float binVal = hist.at<float>(s);
    cv::rectangle(img, cv::Point(s * scale, height - height * (binVal / maxVal)), cv::Point((s + 1) * scale, height),
                  cv::Scalar::all(255), CV_FILLED);
  }
}

std::string cvtype2str(int type)
{
  std::ostringstream oss;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  int chans = 1 + (type >> CV_CN_SHIFT);

  switch (depth)
  {
    case CV_8U:
      oss << "8U";
      break;
    case CV_8S:
      oss << "8S";
      break;
    case CV_16U:
      oss << "16U";
      break;
    case CV_16S:
      oss << "16S";
      break;
    case CV_32S:
      oss << "32S";
      break;
    case CV_32F:
      oss << "32F";
      break;
    case CV_64F:
      oss << "64F";
      break;
    default:
      oss << "User";
      break;
  }

  oss << "C";
  oss << chans;

  return oss.str();
}

std::string describeImage(const cv::Mat& m)
{
  std::ostringstream oss;
  oss << cvtype2str(m.type()) << "," << m.cols << "x" << m.rows;
  return oss.str();
}

std::vector<cv::Point> usedPoints(const cv::Mat& m)
{
  return usedNeighborhood(m, cv::Point(0, 0), 0, m.rows - 1, 0, m.cols - 1);
}

std::vector<cv::Point> usedNeighborhood(const cv::Mat& m, const cv::Point& center, int dRow, int dCol)
{
  return usedNeighborhood(m, center, dRow, dRow, dCol, dCol);
}

std::vector<cv::Point> usedNeighborhood(const cv::Mat& m, const cv::Point& center, int dRow1, int dRow2, int dCol1,
                                        int dCol2)
{
  std::vector<cv::Point> neighbors;
  // Ensuring that bound are respected
  int minRow = std::max(center.y - dRow1, 0);
  int minCol = std::max(center.x - dCol1, 0);
  int maxRow = std::min(center.y + dRow2, m.rows);
  int maxCol = std::min(center.x + dCol2, m.cols);
  for (int row = minRow; row <= maxRow; row++)
  {
    for (int col = minCol; col <= maxCol; col++)
    {
      unsigned char val = m.at<unsigned char>(row, col);
      if (val != 0)
        neighbors.push_back(cv::Point(col, row));
    }
  }
  return neighbors;
}

std::vector<cv::Point> getPointsInConvexHull(const std::vector<cv::Point>& points, const cv::Mat& img)
{
  std::vector<cv::Point> hull;
  convexHull(points, hull);
  std::vector<cv::Point> result;
  for (int row = 0; row < img.rows; row++)
  {
    for (int col = 0; col < img.cols; col++)
    {
      cv::Point p(col, row);
      if (pointPolygonTest(hull, p, false) >= 0)
      {
        result.push_back(p);
      }
    }
  }
  return result;
}

cv::Point3f eigenToCV(Eigen::Vector3d& vector)
{
  return cv::Point3f(vector(0), vector(1), vector(2));
}

}  // namespace Utils
}  // namespace Vision
