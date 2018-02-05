#include "GoalFactory.hpp"

#include "GoalByDNN.hpp"
#include "GoalByII.hpp"

#include "../FilterFactory.hpp"

namespace Vision {
namespace Filters {
void registerGoalFilters(FilterFactory * ff) {
  ff->registerBuilder("GoalByDNN", [](){return std::unique_ptr<Filter>(new GoalByDNN);});
  ff->registerBuilder("GoalByII" , [](){return std::unique_ptr<Filter>(new GoalByII );});
}
}
}
