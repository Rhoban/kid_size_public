#include "CompassProvider.hpp"
#include <string>
#include <vector>

namespace Vision
{
namespace Filters
{
CompassProvider::CompassProvider(const std::string& name) : Filter(name)
{
}

const std::vector<double>& CompassProvider::getCompasses() const
{
  return compasses;
}
const std::vector<double>& CompassProvider::getDispersions() const
{
  return dispersions;
}

void CompassProvider::pushCompass(double angle, double dispersion)
{
  compasses.push_back(angle);
  dispersions.push_back(dispersion);
}

void CompassProvider::clearCompassData()
{
  compasses.clear();
  dispersions.clear();
}

}  // namespace Filters
}  // namespace Vision
