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
void registerBasicsFilters() {
  FilterFactory::registerClass<AbsDiff>("AbsDiff");
  FilterFactory::registerClass<Blur>("Blur");
  FilterFactory::registerClass<Diff>("Diff");
  FilterFactory::registerClass<Dilate>("Dilate");
  FilterFactory::registerClass<Erode>("Erode");
  FilterFactory::registerClass<Invert>("Invert");
  FilterFactory::registerClass<Mask>("Mask");
  FilterFactory::registerClass<Mult>("Mult");
  FilterFactory::registerClass<Norm>("Norm");
  FilterFactory::registerClass<Rescale>("Rescale");
  FilterFactory::registerClass<Threshold>("Threshold");
  FilterFactory::registerClass<Undistort>("Undistort");
  FilterFactory::registerClass<Histogram>("Histogram");
  FilterFactory::registerClass<MaskOr>("MaskOr");
  FilterFactory::registerClass<Integral>("Integral");
}
}
}
