#include "ObstacleFactory.hpp"

#include "ObstacleByDNN.hpp"
#include "ObstacleByII.hpp"

#include "../FilterFactory.hpp"

namespace Vision {
namespace Filters {
void registerObstacleFilters(FilterFactory * ff) {
  ff->registerBuilder("ObstacleByDNN", [](){return std::unique_ptr<Filter>(new ObstacleByDNN);});
  ff->registerBuilder("ObstacleByII" , [](){return std::unique_ptr<Filter>(new ObstacleByII );});
}
}
}
