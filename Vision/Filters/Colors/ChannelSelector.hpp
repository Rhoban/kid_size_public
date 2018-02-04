#ifndef VISION_FILTERS_CHANNELSELECTOR_HPP
#define VISION_FILTERS_CHANNELSELECTOR_HPP

#include <vector>
#include "Filters/Filter.hpp"

namespace Vision {
namespace Filters {

/**
 * ChannelSelector
 *
 * Extract the channel of a given index from the source image
 */
class ChannelSelector : public Filter {
public:
  ChannelSelector() : Filter("ChannelSelector") {}

  /**
   * Initialization with filter name and
   * filter dependency source name
   */
  ChannelSelector(const std::string &name, const std::string &source,
                  Frequency::type frequency = Frequency::Auto);

  virtual std::string getClassName() const override { return "ChannelSelector"; }

protected:
  /**
   * @Inherit
   */
  virtual void process() override;

  virtual void setParameters() override;

private:
  ParamInt channel;
};
}
}

#endif
