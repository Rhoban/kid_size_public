#pragma once

#include <vector>
#include "ObstacleProvider.hpp"

#include "tiny_dnn/tiny_dnn.h"

namespace Vision {
namespace Filters {

/// Simplest filter for detecting the ball
/// - Takes a binary image as input and use the barycenter as a ball candidate
class ObstacleByDNN : public ObstacleProvider {
public:
  ObstacleByDNN();

  virtual std::string getClassName() const;
  virtual Json::Value toJson() const override;
  virtual void fromJson(const Json::Value & v, const std::string & dir_name) override;
  virtual int expectedDependencies() const;

protected:
  virtual void process() override;
  virtual void setParameters() override;

  /// Read weights at 'path' and update the neural network
  void updateNN();

  /// Use the neural network to get the patch score
  double getScore(const cv::Mat & patch);

private:
  /// The neural network used to predict if the patch is a 'ball patch'
  tiny_dnn::network<tiny_dnn::sequential> nn;

  /// The path to the file containing the architecture of the neural network
  /// Warning: Currently, online changes of the path are not available
  std::string arch_path;

  /// The path to the file containing the weights of the neural network
  /// Warning: Currently, online changes of the path are not available
  std::string weights_path;

  /// Debug Level:
  /// 0 - Silent
  /// 1 - Size and score of the roi
  ParamInt debugLevel;

  /// Minimal score for recognizing the ball
  ParamFloat scoreThreshold;
};
}
}
