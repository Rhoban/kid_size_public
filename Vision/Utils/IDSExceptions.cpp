#include "IDSExceptions.hpp"

namespace Vision
{
namespace Utils
{
IDSException::IDSException(const std::string& msg) : std::runtime_error(msg)
{
}

IDSConnectionException::IDSConnectionException(const std::string& msg) : std::runtime_error(msg)
{
}

}  // namespace Utils
}  // namespace Vision
