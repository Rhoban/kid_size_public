#pragma once

#include "rhoban_utils/serialization/json_serializable.h"

#include "Filters/Filter.hpp"

namespace Vision {
namespace Filters {
class FilterFactory {
private:
  typedef std::function<Filter *()> filterCreator;
  typedef std::map<std::string, filterCreator> constructorMap;
  static constructorMap filterClasses;

  static void init();

public:
  static std::vector<std::string> availableFilters();

  template <class T> static void registerClass(const std::string &className) {
    filterClasses[className] = []() { return new T(); };
  }

  static Filter *constructFilter(const std::string &className);

  static Filter *filterFromJson(const Json::Value & v, const std::string & dir_name);
};
}
}
