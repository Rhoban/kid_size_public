#pragma once

#include "Filters/Filter.hpp"

namespace Vision
{
namespace Filters
{
class EverythingByDNN : public Filter
{
public:
  EverythingByDNN() : Filter("EverythingByDNN"), model_path("model.pb")
  {
  }

  virtual std::string getClassName() const override;
  virtual int expectedDependencies() const override;
  virtual Json::Value toJson() const override;
  virtual void fromJson(const Json::Value& v, const std::string& dir_name) override;

protected:
  virtual void process() override;
  virtual void setParameters() override;

  /// Read weights at 'path' and update the neural network
  void updateNN();
  /// Use the neural network to get the patch class and score
  std::pair<int, double> getClass(cv::Mat patch);
  // double getScore(const cv::Mat& patch);

private:
  // /// The number of columns where the 'exact' value is computed
  // ParamInt nbCols;
  // /// The number of rows where the 'exact' value is computed
  // ParamInt nbRows;
  ParamFloat scoreThreshold;
  std::string model_path;
};

}  // namespace Filters
}  // namespace Vision
