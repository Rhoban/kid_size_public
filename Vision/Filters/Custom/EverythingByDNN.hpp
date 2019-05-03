#pragma once

#include "Filters/Filter.hpp"
#include <Filters/Features/FeaturesProvider.hpp>
#include <opencv2/dnn.hpp>

namespace Vision
{
namespace Filters
{
class EverythingByDNN : public Filter, public FeaturesProvider
{
public:
  EverythingByDNN();

  virtual std::string getClassName() const override;
  virtual int expectedDependencies() const override;
  virtual Json::Value toJson() const override;
  virtual void fromJson(const Json::Value& v, const std::string& dir_name) override;

protected:
  virtual void process() override;
  virtual void setParameters() override;
  void updateUsedClasses();

  void updateNN();

  /// Use the neural network to get the patch class and score
  std::pair<int, double> getClass(cv::Mat patch);

private:
  ParamInt debugLevel;
  std::map<std::string,ParamInt> isUsingFeature;
  ParamInt imSize;
  ParamFloat scoreThreshold;
  std::string model_path;
  cv::Ptr<cv::dnn::Importer> importer;
  cv::dnn::Net net;
  std::vector<std::string> classNames;
  std::vector<std::string> usedClassNames;

  static std::map<std::string, hl_monitoring::Field::POIType> stringToPOIEnum;  // TODO find better name ...
};

}  // namespace Filters
}  // namespace Vision
