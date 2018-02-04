#include "BallProvider.hpp"

using namespace rhoban_geometry;

namespace Vision {
namespace Filters {

BallProvider::BallProvider(const std::string &name) : Filter(name) {}

std::vector<double> BallProvider::getBallsX() const
{
  std::vector<double> ballsX;
  for (const Circle & c : balls)
  {
    ballsX.push_back(c.getCenter().x);
  }
  return ballsX;
}

std::vector<double> BallProvider::getBallsY() const
{
  std::vector<double> ballsY;
  for (const Circle & c : balls)
  {
    ballsY.push_back(c.getCenter().y);
  }
  return ballsY;
}

std::vector<double> BallProvider::getBallsRadius() const
{
  std::vector<double> ballsRadius;
  for (const Circle & c : balls)
  {
    ballsRadius.push_back(c.getRadius());
  }
  return ballsRadius;
}

const std::vector<Circle> & BallProvider::getBalls() const
{
  return balls;
}

const std::vector<Circle> & BallProvider::getCandidates() const
{
  return candidates;
}


std::vector<Circle> BallProvider::getBallsScaled(const cv::Mat & img) const
{
  std::vector<Circle> balls_scaled;
  for (const Circle & c : balls)
  {
    balls_scaled.push_back(scaleToImg(c, img));
  }
  return balls_scaled;
}

std::vector<Circle> BallProvider::getCandidatesScaled(const cv::Mat & img) const
{
  std::vector<Circle> candidates_scaled;
  for (const Circle & c : candidates)
  {
    candidates_scaled.push_back(scaleToImg(c, img));
  }
  return candidates_scaled;
}

Circle BallProvider::rescaleCircle(const Circle &circle,
                                   const cv::Mat &circle_img) const
{
  double new_x, new_y, new_radius;
  new_x = circle.getCenter().x / circle_img.cols;
  new_y = circle.getCenter().y / circle_img.rows;
  new_radius = circle.getRadius() / circle_img.cols;
  return Circle(Point(new_x, new_y), new_radius);
}

Circle BallProvider::scaleToImg(const Circle &circle,
                                const cv::Mat &circle_img) const
{
  double new_x, new_y, new_radius;
  new_x = circle.getCenter().x * circle_img.cols;
  new_y = circle.getCenter().y * circle_img.rows;
  new_radius = circle.getRadius() * circle_img.cols;
  return Circle(Point(new_x, new_y), new_radius);
}

void BallProvider::pushBall(const Circle &circle,
                            const cv::Mat &circle_img)
{
  balls.push_back(rescaleCircle(circle, circle_img));
}

void BallProvider::pushBalls(const std::vector<Circle> &circles,
                             const cv::Mat &circle_img) {
  for (const Circle &c : circles) {
    pushBall(c, circle_img);
  }
}

void BallProvider::pushCandidate(const Circle &circle,
                                 const cv::Mat &circle_img)
{
  candidates.push_back(rescaleCircle(circle, circle_img));
}

void BallProvider::pushCandidates(const std::vector<Circle> &circles,
                                  const cv::Mat &circle_img) {
  for (const Circle &c : circles) {
    pushCandidate(c, circle_img);
  }
}

void BallProvider::clearBallsData() {
  balls.clear();
  candidates.clear();
}
}
}
