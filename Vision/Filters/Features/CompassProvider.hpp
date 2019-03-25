#pragma once

#include "Filters/Filter.hpp"
#include <string>
#include <vector>

namespace Vision
{
namespace Filters
{
/// This class describe the architecture of a goal provider and some common code
class CompassProvider : public Filter
{
public:
  CompassProvider(const std::string& name);

  const std::vector<double>& getCompasses() const;    // orientation in rad
  const std::vector<double>& getDispersions() const;  // dispersion of the markers (quality, 0=>complete dispersion
                                                      // 1=>no dispersion)

protected:
  void pushCompass(double angle, double dispersion);

  /// Remove previously published data for the compass
  void clearCompassData();

  /// orientations
  std::vector<double> compasses;
  std::vector<double> dispersions;
};
}  // namespace Filters
}  // namespace Vision
