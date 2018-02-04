#pragma once

#include "Filters/Filter.hpp"

namespace Vision {
namespace Filters {

/// This class describe the architecture of a goal provider and some common code
class GoalProvider : public Filter {
public:
  GoalProvider(const std::string &name);

  /// Values are in [0,1]
  const std::vector<double> & getGoalsX() const;
  /// Values are in [0,1]
  const std::vector<double> & getGoalsY() const;

protected:

  /// Automatically rescale provided point to fit the standard
  void pushGoal(double x, double y, const cv::Mat & goal_img);

  /// Remove previously published data for the goal
  void clearGoalsData();

  /// Position of the goal in the image [0,1]
  std::vector<double> goals_x;

  /// Position of the goal in the image [0,1]
  std::vector<double> goals_y;
};
}
}
