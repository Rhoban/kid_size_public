#pragma once

#include "Filters/Filter.hpp"
#include <opencv2/dnn.hpp>

namespace Vision
{
namespace Filters
{
class EverythingByDNN : public Filter
{
public:
  EverythingByDNN() : Filter("EverythingByDNN"), model_path("model.pb")
  {
    // TODO load classes from json config file
    classNames.push_back("Empty");
    classNames.push_back("Ball");
    classNames.push_back("PostBase");
    classNames.push_back("LineCorner");
    classNames.push_back("PenaltyMark");
    classNames.push_back("Center");
    classNames.push_back("X");
    classNames.push_back("ArenaCorner");
    classNames.push_back("T");
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
  cv::Ptr<cv::dnn::Importer> importer;
  cv::dnn::Net net;
  std::vector<std::string> classNames;
};

}  // namespace Filters
}  // namespace Vision
