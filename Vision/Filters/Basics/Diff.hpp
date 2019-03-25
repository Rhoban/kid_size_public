#pragma once

#include "Filters/Filter.hpp"

namespace Vision
{
namespace Filters
{
/**
 * Diff
 *
 * Calculates the differences between two images A and B
 * (Order matters and if A[i,j] < B[i,j], then R[i,j] = 0)
 */
class Diff : public Filter
{
public:
  Diff() : Filter("Diff")
  {
  }

  virtual std::string getClassName() const override
  {
    return "Diff";
  }
  virtual int expectedDependencies() const override
  {
    return 2;
  }

protected:
  /**
   * @Inherit
   */
  virtual void process() override;
};
}  // namespace Filters
}  // namespace Vision
