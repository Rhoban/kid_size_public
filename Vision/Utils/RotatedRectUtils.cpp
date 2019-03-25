/*************************************************************************
 *  File Name	: 'RotatedRectUtils.cpp'
 *  Author	: Remi FABRE
 *  Contact      : remi.fabre@labri.fr
 *  Created	: dimanche, juin 28 2015
 *  Licence	: http://creativecommons.org/licenses/by-nc-sa/3.0/
 *
 *  Notes:
 *************************************************************************/

#include "RotatedRectUtils.hpp"

#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
using namespace cv;

/**
 * Draws a RotatedRect. Can be a plain drawing of thickness is negative
 */
void drawRotatedRectangle(cv::Mat& img, const cv::RotatedRect& rect, const cv::Scalar& color, int thickness,
                          int lineType)
{
  if (thickness < 0)
  {
    // Plain drawing :
    cv::Point2f rpoints[4];
    rect.points(rpoints);
    cv::Point rect_points[1][4];
    for (int np = 0; np < 4; np++)
    {
      rect_points[0][np] = cv::Point(rpoints[np].x, rpoints[np].y);
    }
    const cv::Point* ppt[1] = { rect_points[0] };
    int npt[] = { 4 };
    cv::fillPoly(img, ppt, npt, 1, color, lineType);
  }
  else
  {
    // Drawing with lines :
    cv::Point2f rectPoints[4];
    rect.points(rectPoints);
    for (int j = 0; j < 4; j++)
    {
      cv::line(img, rectPoints[j], rectPoints[(j + 1) % 4], color, thickness, lineType);
    }
  }
}

void drawRotatedRectangles(cv::Mat& img, const std::vector<cv::RotatedRect>& rects, const cv::Scalar& color,
                           int thickness, int lineType)
{
  for (const cv::RotatedRect& rect : rects)
    drawRotatedRectangle(img, rect, color, thickness, lineType);
}

double getSmallSide(const cv::RotatedRect& rect)
{
  return std::min(rect.size.width, rect.size.height);
}

double getBigSide(const cv::RotatedRect& rect)
{
  return std::max(rect.size.width, rect.size.height);
}

double getAspectRatio(const cv::RotatedRect& rect)
{
  double small = getSmallSide(rect);
  double big = getBigSide(rect);
  if (big <= 0)
    return -1;
  return small / big;
}
