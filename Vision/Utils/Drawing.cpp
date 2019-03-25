#include "Drawing.hpp"

#include "CameraState/CameraState.hpp"

#include "Utils/Interface.h"

#include <opencv2/imgproc.hpp>

using namespace rhoban_geometry;

namespace Vision
{
namespace Utils
{
void draw(cv::Mat& img, const Ellipse& e, const cv::Scalar& color, int thickness)
{
  cv::Size s(e.getBigRadius(), e.getSmallRadius());
  cv::ellipse(img, rg2cv(e.getCenter()), s, e.getAngle().getValue(), 0.0, 360.0, color, thickness);
}

void draw(cv::Mat& img, const ParametricLine& line, const cv::Scalar& color, int thickness)
{
  // from closest Point to Origin, going to each side with a dist of image
  // diagonal
  Point closestPoint = line.projectPoint(Point(0, 0));
  double diagonalLength = sqrt(img.rows * img.rows + img.cols * img.cols);
  Point dist = line.getDirection() * diagonalLength;
  cv::Point from = rg2cv(closestPoint + dist);
  cv::Point to = rg2cv(closestPoint - dist);
  cv::line(img, from, to, color, thickness);
}

void draw(cv::Mat& img, const std::vector<ParametricLine>& lines, const cv::Scalar& color, int thickness)
{
  for (unsigned int i = 0; i < lines.size(); i++)
  {
    draw(img, lines[i], color, thickness);
  }
}
}  // namespace Utils
}  // namespace Vision
