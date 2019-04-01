#include "Utils/Interface.h"

#include <opencv2/core/eigen.hpp>
#include <opencv2/calib3d.hpp>

#include <cmath>

namespace Vision
{
rhoban_geometry::Point cv2rg(const cv::Point& p)
{
  return rhoban_geometry::Point(p.x, p.y);
}

rhoban_geometry::Point cv2rg(const cv::Point2f& p)
{
  return rhoban_geometry::Point(p.x, p.y);
}

rhoban_geometry::Point cv2rg(const cv::Point3f& p)
{
  return rhoban_geometry::Point(p.x, p.y);
}

cv::Point rg2cv(const rhoban_geometry::Point& p)
{
  return cv::Point((int)p.x, (int)p.y);
}

cv::Point2f rg2cv2f(const rhoban_geometry::Point& p)
{
  return cv::Point2f(p.x, p.y);
}

rhoban_utils::Angle angleBetween(const cv::Point3f& v1, const cv::Point3f& v2)
{
  double nv1 = norm(v1);
  double nv2 = norm(v2);
  if (nv1 < 1e-9 || nv2 < 1e-9)
  {
    return 0;
  }
  double x = v1.dot(v2) / (nv1 * nv2);
  if (x > 1.0)
  {
    x = 1.0;
  }
  if (x < -1.0)
  {
    x = -1.0;
  }
  return rhoban_utils::Angle::arccos(x);
}

void cvToAffine(const cv::Mat& rvec, const cv::Mat& tvec, Eigen::Affine3d* affine)
{
  cv::Mat rot3d;
  cv::Rodrigues(rvec, rot3d);
  Eigen::Matrix3d orientation;
  Eigen::Vector3d translation;
  cv2eigen(rvec, orientation);
  cv2eigen(tvec, translation);
  *affine = Eigen::Affine3d(orientation) * Eigen::Translation3d(translation);
}

void affineToCV(const Eigen::Affine3d& affine, cv::Mat* rvec, cv::Mat* tvec)
{
  cv::Mat rot3d;
  Eigen::Matrix3d rot_eigen = affine.linear();
  Eigen::Vector3d translation_eigen = affine.translation();
  eigen2cv(rot_eigen, rot3d);
  eigen2cv(translation_eigen, *tvec);
  cv::Rodrigues(rot3d, *rvec);
}

}  // namespace Vision
