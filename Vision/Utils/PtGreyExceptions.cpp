#include "PtGreyExceptions.hpp"

namespace Vision
{

PtGreyException::PtGreyException(const std::string & msg)
  : std::runtime_error(msg) {}

PtGreyConnectionException::PtGreyConnectionException(const std::string & msg)
  : std::runtime_error(msg) {}

}