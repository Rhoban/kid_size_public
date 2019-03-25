#pragma once

#include <stdexcept>

namespace Vision
{
namespace Utils
{
class IDSException : public std::runtime_error
{
public:
  IDSException(const std::string& msg);
};

class IDSConnectionException : public std::runtime_error
{
public:
  IDSConnectionException(const std::string& msg);
};

}  // namespace Utils
}  // namespace Vision
