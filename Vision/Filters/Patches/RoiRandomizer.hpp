#pragma once

#include "Filters/Filter.hpp"

namespace Vision
{
namespace Filters
{
/**
 * dependency[0]: source image (used to extract patches)
 * dependency[1]: use as roi provider
 */
class ROIBasedGTP : public GroundTruthProvider
{
};
}  // namespace Filters
}  // namespace Vision
