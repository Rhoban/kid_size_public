#pragma once

#include "Filters/Filter.hpp"

namespace Vision {
namespace Filters {

/// This class describe the architecture of a obstacle provider and some common code
class ObstacleProvider : public Filter {
public:
  ObstacleProvider(const std::string &name);

  /// Values are in [0,1] x [0,1] (in Image)
  const std::vector<Eigen::Vector2d> & getObstacles() const;

protected:

  /// Automatically rescale provided point to fit the standard
  void pushObstacle(double x, double y, const cv::Mat & obstacle_img);

  /// Remove previously published data for the obstacle
  void clearObstaclesData();

  /// Position of the obstacle in the image [0,1]x[0,1]
  std::vector<Eigen::Vector2d> obstacles;
};
}
}
