#include "BasicsFactory.hpp"

#include "AbsDiff.hpp"
#include "Blur.hpp"
#include "Diff.hpp"
#include "Dilate.hpp"
#include "Erode.hpp"
#include "Invert.hpp"
#include "Mask.hpp"
#include "Mult.hpp"
#include "Norm.hpp"
#include "Rescale.hpp"
#include "Threshold.hpp"
#include "Undistort.hpp"
#include "Histogram.hpp"
#include "MaskOr.hpp"
#include "Integral.hpp"

#include "../FilterFactory.hpp"

namespace Vision {
namespace Filters {
void registerBasicsFilters(FilterFactory * ff) {
  ff->registerBuilder("AbsDiff"  , [](){return std::unique_ptr<Filter>(new AbsDiff  );});
  ff->registerBuilder("Blur"     , [](){return std::unique_ptr<Filter>(new Blur()     );});
  ff->registerBuilder("Diff"     , [](){return std::unique_ptr<Filter>(new Diff()     );});
  ff->registerBuilder("Dilate"   , [](){return std::unique_ptr<Filter>(new Dilate()   );});
  ff->registerBuilder("Erode"    , [](){return std::unique_ptr<Filter>(new Erode()    );});
  ff->registerBuilder("Invert"   , [](){return std::unique_ptr<Filter>(new Invert()   );});
  ff->registerBuilder("Mask"     , [](){return std::unique_ptr<Filter>(new Mask()     );});
  ff->registerBuilder("Mult"     , [](){return std::unique_ptr<Filter>(new Mult()     );});
  ff->registerBuilder("Norm"     , [](){return std::unique_ptr<Filter>(new Norm()     );});
  ff->registerBuilder("Rescale"  , [](){return std::unique_ptr<Filter>(new Rescale()  );});
  ff->registerBuilder("Threshold", [](){return std::unique_ptr<Filter>(new Threshold());});
  ff->registerBuilder("Undistort", [](){return std::unique_ptr<Filter>(new Undistort());});
  ff->registerBuilder("Histogram", [](){return std::unique_ptr<Filter>(new Histogram());});
  ff->registerBuilder("MaskOr"   , [](){return std::unique_ptr<Filter>(new MaskOr()   );});
  ff->registerBuilder("Integral" , [](){return std::unique_ptr<Filter>(new Integral() );});
}

}
}
