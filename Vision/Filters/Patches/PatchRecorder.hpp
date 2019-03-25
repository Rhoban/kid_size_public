#pragma once

#include "Filters/Filter.hpp"

namespace Vision
{
namespace Filters
{
/// Uses
class PatchRecorder : public Filter
{
public:
  PatchRecorder();

  void setParameters() override;

  virtual std::string getClassName() const override;
  virtual Json::Value toJson() const override;
  virtual void fromJson(const Json::Value& v, const std::string& dir_name) override;

  virtual int expectedDependencies() const;

protected:
  virtual void process() override;

  int imgId;
  std::string prefix;

  /// If period >= 1: save the patches every 'period' images
  /// If period <= 0: never save the patches
  ParamInt period;

  /// Actual position in period
  int curr_period;
};

}  // namespace Filters
}  // namespace Vision
