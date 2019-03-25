#ifndef VISION_DEBUG_LEVEL_HPP
#define VISION_DEBUG_LEVEL_HPP

#include "rhoban_utils/serialization/json_serializable.h"

namespace Vision
{
class DebugLevel : public rhoban_utils::JsonSerializable
{
public:
  bool threads, graphics, parameters, perfs;

  DebugLevel() : threads(false), graphics(true), parameters(true), perfs(false)
  {
  }

  DebugLevel(bool global) : threads(global), graphics(global), parameters(global), perfs(global)
  {
  }

  inline void disableAll()
  {
    threads = false;
    graphics = false;
    parameters = false;
    perfs = false;
  }

  inline bool enabled()
  {
    return threads || graphics || parameters || perfs;
  }

  virtual Json::Value toJson() const override;
  virtual void fromJson(const Json::Value& v, const std::string& dir_name);
  virtual std::string getClassName() const override
  {
    return "DebugLevel";
  }
};
}  // namespace Vision

#endif  // VISION_DEBUG_LEVEL_HPP
