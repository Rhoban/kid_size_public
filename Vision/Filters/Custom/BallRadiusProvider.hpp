#pragma once

#include "Filters/Filter.hpp"

namespace Vision
{
namespace Filters
{
/// Generate a mono-canal image of the same size that the source image
/// - for each pixel, the value is the expected radius of the ball inside the image
/// In order to spare computing power, only a subset of the required value is
/// computed. Values between those points are obtained through interpolation.
class BallRadiusProvider : public Filter
{
public:
  BallRadiusProvider() : Filter("BallRadiusProvider")
  {
  }

  virtual std::string getClassName() const override;
  virtual int expectedDependencies() const override;

protected:
  virtual void process() override;
  virtual void setParameters() override;

private:
  /// The number of columns where the 'exact' value is computed
  ParamInt nbCols;
  /// The number of rows where the 'exact' value is computed
  ParamInt nbRows;
};

}  // namespace Filters
}  // namespace Vision
