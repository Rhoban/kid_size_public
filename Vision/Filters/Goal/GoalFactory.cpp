#include "GoalFactory.hpp"

#include "GoalByDNN.hpp"
#include "GoalByII.hpp"

#include "../FilterFactory.hpp"

namespace Vision {
namespace Filters {
void registerGoalFilters() {
  FilterFactory::registerClass<GoalByDNN>("GoalByDNN");
  FilterFactory::registerClass<GoalByII>("GoalByII");
}
}
}
