#include "GoalProvider.hpp"

namespace Vision {
namespace Filters {

GoalProvider::GoalProvider(const std::string &name) : Filter(name) {}

const std::vector<double> & GoalProvider::getGoalsX() const
{
  return goals_x;
}

const std::vector<double> & GoalProvider::getGoalsY() const
{
  return goals_y;
}

void GoalProvider::pushGoal(double x, double y,
                            const cv::Mat & goal_img)
{
  double new_x, new_y;
  new_x = x / goal_img.cols;
  new_y = y / goal_img.rows;
  goals_x.push_back(new_x);
  goals_y.push_back(new_y);
}

void GoalProvider::clearGoalsData() {
  goals_x.clear();
  goals_y.clear();
}

}
}
