#pragma once

#include <opencv2/core/core.hpp>

#include "rhoban_geometry/parametric_line.h"
#include "rhoban_geometry/ellipse.h"

namespace Vision
{
namespace Utils
{
void draw(cv::Mat& img, const rhoban_geometry::Ellipse& e, const cv::Scalar& color, int thickness = 1);

void draw(cv::Mat& img, const rhoban_geometry::ParametricLine& p, const cv::Scalar& color, int thickness = 1);

void draw(cv::Mat& img, const std::vector<rhoban_geometry::ParametricLine>& lines, const cv::Scalar& color,
          int thickness = 1);
}  // namespace Utils
}  // namespace Vision
