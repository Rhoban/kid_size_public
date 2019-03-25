#pragma once

#include "Filters/Filter.hpp"

namespace Vision
{
namespace Filters
{
/// Write images too disk:
/// - write only images when enabled becomes false
/// TODO: add a maximal length
class Recorder : public Filter
{
public:
  /// Initially not recording
  Recorder();

  virtual ~Recorder();

  virtual std::string getClassName() const override;

protected:
  virtual void process() override;

  /// Lazy mechanism
  void initControl();

  void updateControl();

private:
  bool activated;

  /// Dumping images directly consumes too much time
  std::vector<cv::Mat> memory;

  int image_no;
};
}  // namespace Filters
}  // namespace Vision
