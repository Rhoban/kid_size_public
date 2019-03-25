#pragma once

#include <stdexcept>

namespace Vision
{
namespace Utils
{
/// StreamEndException happens when trying to go before first image or after
/// last image in a fixed sequence of images
class StreamEndException : public std::out_of_range
{
public:
  StreamEndException(const std::string& what) : std::out_of_range(what)
  {
  }
};

}  // namespace Utils
}  // namespace Vision
