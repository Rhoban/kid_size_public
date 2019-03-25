#pragma once

#include <opencv2/core/core.hpp>

#include "rhoban_utils/angle.h"

namespace Vision
{
namespace Utils
{
class HT
{
public:
  static cv::Mat rotX(const rhoban_utils::Angle& theta);
  static cv::Mat rotY(const rhoban_utils::Angle& theta);
  static cv::Mat rotZ(const rhoban_utils::Angle& theta);
};
}  // namespace Utils
}  // namespace Vision
