#include "CustomFactory.hpp"

#include "BallRadiusProvider.hpp"
#include "Recorder.hpp"
#include "ColorDensity.hpp"
#include "FieldBorder.hpp"
#include "ClippingByBorder.hpp"
#include "WhiteLines.hpp"
#include "RobotByII.hpp"
#include "../FilterFactory.hpp"


namespace Vision {
namespace Filters {
void registerCustomFilters() {
  FilterFactory::registerClass<BallRadiusProvider>("BallRadiusProvider");
  FilterFactory::registerClass<Recorder>("Recorder");
  FilterFactory::registerClass<ColorDensity>("ColorDensity");
  FilterFactory::registerClass<FieldBorder>("FieldBorder");
  FilterFactory::registerClass<ClippingByBorder>("ClippingByBorder");
  FilterFactory::registerClass<WhiteLines>("WhiteLines");
  FilterFactory::registerClass<RobotByII>("RobotByII");
}
}
}
