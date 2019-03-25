#pragma once

#include "Filters/Source/Source.hpp"

#include <opencv2/opencv.hpp>

namespace Vision
{
namespace Filters
{
/**
 * SourceOpencv
 *
 * Very basic support to open cameras using openCV
 */
class SourceOpenCV : public Source
{
public:
  SourceOpenCV();
  virtual ~SourceOpenCV();

  std::string getClassName() const override;
  int expectedDependencies() const override;

  void fromJson(const Json::Value& v, const std::string& dir_name) override;
  Json::Value toJson() const override;

  Type getType() const override;

protected:
  void process() override;

private:
  std::string device_name;
  int device_index;

  cv::VideoCapture capture_device;
};

}  // namespace Filters
}  // namespace Vision
