#include "ObstacleProvider.hpp"

namespace Vision {
namespace Filters {

ObstacleProvider::ObstacleProvider(const std::string &name) : Filter(name) {}

const std::vector<double> & ObstacleProvider::getObstaclesX() const
{
  return obstacles_x;
}

const std::vector<double> & ObstacleProvider::getObstaclesY() const
{
  return obstacles_y;
}

void ObstacleProvider::pushObstacle(double x, double y,
                                    const cv::Mat & obstacle_img)
{
  double new_x, new_y;
  new_x = x / obstacle_img.cols;
  new_y = y / obstacle_img.rows;
  obstacles_x.push_back(new_x);
  obstacles_y.push_back(new_y);
}

void ObstacleProvider::clearObstaclesData() {
  obstacles_x.clear();
  obstacles_y.clear();
}

}
}
