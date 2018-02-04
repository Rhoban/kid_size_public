#pragma once

#include "Filters/Filter.hpp"

#include "rhoban_geometry/circle.h"

namespace Vision {
namespace Filters {

/// This class describe the architecture of a ball provider and some common code
class BallProvider : public Filter {
public:
  typedef rhoban_geometry::Circle Circle;
  
  BallProvider(const std::string &name);

  std::vector<double> getBallsX() const;
  std::vector<double> getBallsY() const;
  std::vector<double> getBallsRadius() const;

  const std::vector<Circle> & getBalls() const;
  const std::vector<Circle> & getCandidates() const;

  /// Return the list of balls in the img referential
  std::vector<Circle> getBallsScaled(const cv::Mat & img) const;

  /// Return the list of candidates in the img referential
  std::vector<Circle> getCandidatesScaled(const cv::Mat & img) const;

protected:

  /// Rescale the circle to be image independent
  Circle rescaleCircle(const Circle & circle, const cv::Mat & circle_img) const;

  /// Rescale the circle from [0,1] notation to img size
  Circle scaleToImg(const Circle & circle, const cv::Mat & output_img) const;

  /// Automatically rescale provided circles to fit the standard
  void pushBall(const Circle &circle, const cv::Mat &circle_img);

  /// Automatically rescale provided circles to fit the standard
  void pushBalls(const std::vector<Circle> &circles,
                 const cv::Mat &circle_img);

  /// Automatically rescale provided circles to fit the standard
  void pushCandidate(const Circle &circle, const cv::Mat &circle_img);

  /// Automatically rescale provided circles to fit the standard
  void pushCandidates(const std::vector<Circle> &circles,
                      const cv::Mat &circle_img);

  /// Remove previously published data for the ball
  void clearBallsData();

  /// Which circle have been accepted as balls
  std::vector<Circle> balls;

  /// Which candidates have been proposed
  std::vector<Circle> candidates;
};
}
}
