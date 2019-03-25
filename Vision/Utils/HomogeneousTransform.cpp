#include "HomogeneousTransform.hpp"

using namespace rhoban_utils;

namespace Vision
{
namespace Utils
{
cv::Mat HT::rotX(const Angle& theta)
{
  cv::Mat m = cv::Mat::zeros(4, 4, CV_32F);
  m.at<float>(3, 3) = 1;
  m.at<float>(0, 0) = 1;
  m.at<float>(1, 1) = cos(theta);
  m.at<float>(1, 2) = sin(theta);
  m.at<float>(2, 2) = cos(theta);
  m.at<float>(2, 1) = -sin(theta);
  return m;
}

cv::Mat HT::rotY(const Angle& theta)
{
  cv::Mat m = cv::Mat::zeros(4, 4, CV_32F);
  m.at<float>(3, 3) = 1;
  m.at<float>(1, 1) = 1;
  m.at<float>(0, 0) = cos(theta);
  m.at<float>(0, 2) = -sin(theta);
  m.at<float>(2, 2) = cos(theta);
  m.at<float>(2, 0) = sin(theta);
  return m;
}

cv::Mat HT::rotZ(const Angle& theta)
{
  cv::Mat m = cv::Mat::zeros(4, 4, CV_32F);
  m.at<float>(3, 3) = 1;
  m.at<float>(2, 2) = 1;
  m.at<float>(0, 0) = cos(theta);
  m.at<float>(0, 1) = sin(theta);
  m.at<float>(1, 1) = cos(theta);
  m.at<float>(1, 0) = -sin(theta);
  return m;
}
}  // namespace Utils
}  // namespace Vision
