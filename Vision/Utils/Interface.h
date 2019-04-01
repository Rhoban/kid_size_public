#pragma once

#include <opencv2/core/core.hpp>

#include "rhoban_geometry/point.h"

#include <Eigen/Geometry>

// Pure interface between opencv and rhoban_geometry
namespace Vision
{
rhoban_geometry::Point cv2rg(const cv::Point& p);
rhoban_geometry::Point cv2rg(const cv::Point2f& p);
/**
 * Ignores the last dimension of 'p'
 */
rhoban_geometry::Point cv2rg(const cv::Point3f& p);

cv::Point rg2cv(const rhoban_geometry::Point& p);
cv::Point2f rg2cv2f(const rhoban_geometry::Point& p);
rhoban_utils::Angle angleBetween(const cv::Point3f& v1, const cv::Point3f& v2);

/**
 * Convert from Rodrigues + translation to Eigen::Affine3d
 */
void cvToAffine(const cv::Mat& rvec, const cv::Mat& tvec, Eigen::Affine3d* affine);
/**
 * Convert from Affine3d to Rodrigues and translation
 */
void affineToCV(const Eigen::Affine3d& affine, cv::Mat* rvec, cv::Mat* tvec);

}  // namespace Vision
