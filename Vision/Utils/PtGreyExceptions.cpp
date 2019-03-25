#include "PtGreyExceptions.hpp"

namespace Vision
{
namespace Utils
{
PtGreyException::PtGreyException(const std::string& msg) : std::runtime_error(msg)
{
}

PtGreyConnectionException::PtGreyConnectionException(const std::string& msg) : std::runtime_error(msg)
{
}

}  // namespace Utils
}  // namespace Vision
