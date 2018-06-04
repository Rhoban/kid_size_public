#include "CustomFactory.hpp"

#include "BallRadiusProvider.hpp"
#include "Recorder.hpp"
#include "ColorDensity.hpp"
#include "FieldBorder.hpp"
#include "ClippingByBorder.hpp"
#include "WhiteLines.hpp"
#include "../FilterFactory.hpp"


namespace Vision {
namespace Filters {
void registerCustomFilters(FilterFactory * ff) {
  ff->registerBuilder("BallRadiusProvider", [](){return std::unique_ptr<Filter>(new BallRadiusProvider);});
  ff->registerBuilder("Recorder"          , [](){return std::unique_ptr<Filter>(new Recorder()          );});
  ff->registerBuilder("ColorDensity"      , [](){return std::unique_ptr<Filter>(new ColorDensity()      );});
  ff->registerBuilder("FieldBorder"       , [](){return std::unique_ptr<Filter>(new FieldBorder()       );});
  ff->registerBuilder("ClippingByBorder"  , [](){return std::unique_ptr<Filter>(new ClippingByBorder()  );});
  ff->registerBuilder("WhiteLines"        , [](){return std::unique_ptr<Filter>(new WhiteLines()        );});
}
}
}
