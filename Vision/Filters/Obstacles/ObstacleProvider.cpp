#include "ObstacleProvider.hpp"

#include "CameraState/CameraState.hpp"

namespace Vision {
namespace Filters {

ObstacleProvider::ObstacleProvider(const std::string &name) : Filter(name) {}

const std::vector<Eigen::Vector2d> &ObstacleProvider::getObstacles() const { return obstacles; }

void ObstacleProvider::pushObstacle(double x, double y, const cv::Mat &obstacle_img) {
  double new_x, new_y;
  new_x = x / obstacle_img.cols * getCS().getCameraModel().getImgWidth();
  new_y = y / obstacle_img.rows * getCS().getCameraModel().getImgHeight();
  obstacles.push_back(Eigen::Vector2d(new_x, new_y));
}

void ObstacleProvider::clearObstaclesData() { obstacles.clear(); }

}  // namespace Filters
}  // namespace Vision
