#include "CorridorProfile.hpp"

#include <rhoban_utils/util.h>

#include <iostream>

CorridorProfile::CorridorProfile() {
  x_limits = {};
  weights = {1,1};
}

double CorridorProfile::getWeight(double x, double y) {
  int x_idx = x_limits.size()-1;
  while (x_idx >= 0  && x < x_limits[x_idx]) {
    x_idx--;
  }
  if (x_idx >= 0) {
    return weights[weights.size() - x_limits.size() + x_idx];
  }
  // we assume that y_limits is sorted
  size_t idx = 0;
  while (idx < y_limits.size()) {
    if (y < y_limits[idx]) break;
    idx++;
  }
  return weights[idx];
}
std::string CorridorProfile::getClassName() const {
  return "CorridorProfile";
}
Json::Value CorridorProfile::toJson() const {
  Json::Value v;
  v["x_limits"] =  rhoban_utils::vector2Json(x_limits);
  v["y_limits"] = rhoban_utils::vector2Json(y_limits);
  v["weights"]  = rhoban_utils::vector2Json(weights);
  return v;
}
void CorridorProfile::fromJson(const Json::Value & v,
                               const std::string & dir_name) {
  (void) dir_name;
  x_limits = rhoban_utils::readVector<double>(v,"x_limits");
  y_limits = rhoban_utils::readVector<double>(v,"y_limits");
  weights = rhoban_utils::readVector<double>(v,"weights");

  if (x_limits.size()+y_limits.size()+1 != weights.size()) {
    throw std::runtime_error(DEBUG_INFO + " incompatible sizes for limits and weights");
  }
}
