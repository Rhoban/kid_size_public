#include "BallFactory.hpp"

#include "BallByII.hpp"

#include "../FilterFactory.hpp"

namespace Vision
{
namespace Filters
{
void registerBallFilters(FilterFactory* ff)
{
  ff->registerBuilder("BallByII", []() { return std::unique_ptr<Filter>(new BallByII()); });
}

}  // namespace Filters
}  // namespace Vision
