#pragma once

#include "Filters/Filter.hpp"

namespace Vision
{
namespace Filters
{
/**
 * Hisogram
 *
 * Separates the input image in 3 channels and draws an histogram for each of
 *them
 */
class Histogram : public Filter
{
public:
  Histogram();

  virtual std::string getClassName() const override
  {
    return "Histogram";
  }

protected:
  /**
   * @Inherit
   */
  virtual void process() override;

  virtual void setParameters() override;

private:
  ParamInt bins;
};
}  // namespace Filters
}  // namespace Vision
