#pragma once

#include <opencv2/core/core.hpp>

#include "rhoban_geometry/point.h"

// Pure interface between opencv and rhoban_geometry
namespace Vision
{

rhoban_geometry::Point cv2rg(const cv::Point & p);
rhoban_geometry::Point cv2rg(const cv::Point2f & p);

cv::Point rg2cv(const rhoban_geometry::Point & p);
cv::Point2f rg2cv2f(const rhoban_geometry::Point & p);
rhoban_utils::Angle angleBetween(const cv::Point3f & v1, const cv::Point3f & v2);

}
