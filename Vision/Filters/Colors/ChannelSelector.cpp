#include <opencv2/core/core.hpp>
#include "Filters/Colors/ChannelSelector.hpp"

namespace Vision
{
namespace Filters
{
void ChannelSelector::setParameters()
{
  channel = ParamInt(0, 0, 2);
  params()->define<ParamInt>("channel", &channel);
}

void ChannelSelector::process()
{
  cv::Mat src = *(getDependency().getImg());
  std::vector<cv::Mat> channels;
  split(src, channels);
  if (channel >= (int)channels.size())
  {
    std::ostringstream oss;
    oss << "Asking for channel '" << channel << "' while image has " << channels.size() << " channels.";
    throw std::runtime_error(oss.str());
  }
  img() = channels[channel];
}
}  // namespace Filters
}  // namespace Vision
