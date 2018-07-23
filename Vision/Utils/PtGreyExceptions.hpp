#pragma once

#include <stdexcept>

namespace Vision
{

class PtGreyException : public std::runtime_error {
public:
  PtGreyException(const std::string & msg);
};

class PtGreyConnectionException : public std::runtime_error {
public:
  PtGreyConnectionException(const std::string & msg);
};

}