#include "FilterFactory.hpp"

#include "Ball/BallFactory.hpp"
#include "Basics/BasicsFactory.hpp"
#include "Colors/ColorsFactory.hpp"
#include "Custom/CustomFactory.hpp"
#include "Features/FeaturesFactory.hpp"
#include "Goal/GoalFactory.hpp"
#include "Patches/PatchFactory.hpp"
#include "Source/SourceFactory.hpp"

#include <exception>
#include <string>
#include <vector>

namespace Vision {
namespace Filters {

FilterFactory::constructorMap FilterFactory::filterClasses;

std::vector<std::string> FilterFactory::availableFilters() {
  if (filterClasses.size() == 0)
    init();
  std::vector<std::string> result;
  for (const auto &pair : filterClasses) {
    result.push_back(pair.first);
  }
  return result;
}

Filter *FilterFactory::constructFilter(const std::string &className) {
  if (filterClasses.size() == 0)
    init();
  constructorMap::iterator it = filterClasses.find(className);
  if (it == filterClasses.end())
    throw std::runtime_error("Class is not registered as a Filter : " +
                             className);
  Filter *newFilter = it->second();
  for (int i = 0; i < newFilter->expectedDependencies(); i++) {
    newFilter->addDependency("?");
  }
  return newFilter;
}

Filter *FilterFactory::filterFromJson(const Json::Value & v, const std::string & dir_name) {
  std::string className;
  rhoban_utils::tryRead(v,"className",&className);
  Filter *newFilter = constructFilter(className);
  newFilter->fromJson(v, dir_name);
  return newFilter;
}

void FilterFactory::init() {
  registerBallFilters();
  registerBasicsFilters();
  registerColorsFilters();
  registerCustomFilters();
  registerFeaturesFilters();
  registerGoalFilters();
  registerPatchFilters();
  registerSourceFilters();
}
}
}
