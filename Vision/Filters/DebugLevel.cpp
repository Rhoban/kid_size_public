#include "DebugLevel.hpp"

namespace Vision
{
Json::Value DebugLevel::toJson() const
{
  Json::Value v;
  v["threads"] = threads;
  v["graphics"] = graphics;
  v["parameters"] = parameters;
  v["perfs"] = perfs;
  return v;
}

void DebugLevel::fromJson(const Json::Value& v, const std::string& dir_name)
{
  rhoban_utils::tryRead(v, "threads", &threads);
  rhoban_utils::tryRead(v, "graphics", &graphics);
  rhoban_utils::tryRead(v, "parameters", &parameters);
  rhoban_utils::tryRead(v, "perfs", &perfs);
}
}  // namespace Vision
