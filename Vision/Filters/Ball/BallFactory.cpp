#include "BallFactory.hpp"

#include "BallByDNN.hpp"
#include "BallByII.hpp"

#include "../FilterFactory.hpp"

namespace Vision {
namespace Filters {
void registerBallFilters() {
  FilterFactory::registerClass<BallByDNN>("BallByDNN");
  FilterFactory::registerClass<BallByII>("BallByII");
}
}
}
