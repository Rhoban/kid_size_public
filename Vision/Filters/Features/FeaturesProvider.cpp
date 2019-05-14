#include <Filters/Features/FeaturesProvider.hpp>

namespace Vision
{
namespace Filters
{
FeaturesProvider::FeaturesProvider()
{
}

FeaturesProvider::~FeaturesProvider()
{
}

const std::vector<cv::Point2f>& FeaturesProvider::getBalls() const
{
  return ball_positions;
}

const std::map<hl_monitoring::Field::POIType, std::vector<cv::Point2f>>& FeaturesProvider::getPOIs() const
{
  return poi_positions;
}

const std::vector<cv::Point2f>& FeaturesProvider::getRobots() const
{
  return robot_positions;
}

void FeaturesProvider::clearAllFeatures()
{
  ball_positions.clear();
  poi_positions.clear();
  robot_positions.clear();
}

void FeaturesProvider::pushBall(const cv::Point2f& img_pos)
{
  ball_positions.push_back(img_pos);
}

void FeaturesProvider::pushPOI(hl_monitoring::Field::POIType type, const cv::Point2f& img_pos)
{
  poi_positions[type].push_back(img_pos);
}

void FeaturesProvider::pushRobot(const cv::Point2f& img_pos)
{
  robot_positions.push_back(img_pos);
}

}  // namespace Filters
}  // namespace Vision
