#pragma once

#include <vector>
#include "Filters/Filter.hpp"

namespace Vision
{
namespace Filters
{
/**
 * ChannelSelector
 *
 * Extract the channel of a given index from the source image
 */
class ChannelSelector : public Filter
{
public:
  ChannelSelector() : Filter("ChannelSelector")
  {
  }

  virtual std::string getClassName() const override
  {
    return "ChannelSelector";
  }

protected:
  /**
   * @Inherit
   */
  virtual void process() override;

  virtual void setParameters() override;

private:
  ParamInt channel;
};
}  // namespace Filters
}  // namespace Vision
