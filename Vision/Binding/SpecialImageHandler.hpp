#pragma once

#include <opencv2/core/core.hpp>

#include <functional>

namespace Vision
{
/// Provides all the information required to represent a special image such as
/// TopView, RobotView or TaggedImg
class SpecialImageHandler
{
public:
  typedef std::function<cv::Mat(int width, int height)> Getter;

  SpecialImageHandler();
  SpecialImageHandler(const std::string& name, int width, int height, Getter getter, double scale = 1.0,
                      bool display = false);

  int getSize() const;
  int getWidth() const;
  int getHeight() const;

  std::string name;
  int baseWidth, baseHeight;  // Original size
  double scale;               // Relative to img base size
  bool display;
  cv::Mat lastImg;
  Getter getter;
};
}  // namespace Vision
