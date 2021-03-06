#include "FilterFactory.hpp"

#include "Ball/BallFactory.hpp"
#include "Basics/BasicsFactory.hpp"
#include "Colors/ColorsFactory.hpp"
#include "Custom/CustomFactory.hpp"
#include "Features/FeaturesFactory.hpp"
#include "Patches/PatchFactory.hpp"
#include "Source/SourceFactory.hpp"

#include <exception>
#include <string>
#include <vector>

namespace Vision
{
namespace Filters
{
FilterFactory::FilterFactory()
{
  registerBallFilters(this);
  registerBasicsFilters(this);
  registerColorsFilters(this);
  registerCustomFilters(this);
  registerFeaturesFilters(this);
  registerPatchFilters(this);
  registerSourceFilters(this);
}

}  // namespace Filters
}  // namespace Vision
