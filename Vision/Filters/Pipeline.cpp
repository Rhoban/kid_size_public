#include <stdexcept>
#include <list>
#include <functional>
#include <fstream>
#include "Filters/Pipeline.hpp"
#include <iostream>
#include "Filters/FilterFactory.hpp"
#include "CameraState/CameraState.hpp"

#include "rhoban_utils/timing/benchmark.h"

using Vision::Utils::CameraState;
using ::rhoban_utils::TimeStamp;

using namespace std;
using namespace rhoban_utils;

namespace Vision {

Pipeline::Pipeline()
  : _filters(), _rootFilters(), _children(), _filterThread(), _timestamp() {
  cs = NULL;
}

Pipeline::~Pipeline() {
  // Free Filters
  _rootFilters.clear();
  _children.clear();
  for (auto &it : _filters) {
    delete it.second;
  }
  _filters.clear();
}

Utils::CameraState *Pipeline::getCameraState() { return cs; }
void Pipeline::setCameraState(Utils::CameraState *csInit) { cs = csInit; }

void Pipeline::add(Filter *filter) {
  if (filter == nullptr) {
    throw std::logic_error("Pipeline null filter");
  }
  if (_filters.count(filter->getName()) != 0) {
    throw std::logic_error("Pipeline filter name already register: '" +
                           filter->getName() + "'");
  }

  _filters[filter->getName()] = filter;
  filter->_pipeline = this;

  if (filter->_dependencies.size() == 0) {
    _rootFilters.push_back(filter);
  }
}

void Pipeline::add(std::vector<std::unique_ptr<Filter>> * filters) {
  for (size_t idx = 0; idx < filters->size(); idx++) {
    add((*filters)[idx].release());
  }
}

const Filter &Pipeline::get(const std::string &name) const {
  try {
    return *(_filters.at(name));
  } catch (const std::out_of_range &e) {
    throw std::runtime_error("No filters named '" + name +
                             "' found in pipeline");
  }
}
Filter &Pipeline::get(const std::string &name) {
  try {
    return *(_filters.at(name));
  } catch (const std::out_of_range &e) {
    cout << "No filters named '" + name + "' found in pipeline" << endl;
    throw std::runtime_error("No filters named '" + name +
                             "' found in pipeline");
  }
}

bool Pipeline::isFilterPresent(const std::string &name) {
  return _filters.find(name) != _filters.end();
}

const Pipeline::FiltersMap &Pipeline::filters() const { return _filters; }
Pipeline::FiltersMap &Pipeline::filters() { return _filters; }

void Pipeline::step(Filter::UpdateType updateType) {
  Benchmark::open("Resolve dependencies");
  resolveDependencies();
  Benchmark::close("Resolve dependencies");

  // Apply previous on nonDependency filters
  std::list<Filter *> list;
  std::map<std::string, int> dependenciesSolved;
  for (Filter *filter : _rootFilters) {

    filter->runStep(updateType);

    if (filter->name == "source") {
      // Vital step : after processing the source filter, the timestamp of the
      // image is known
      // and the cameraState needs to be updated. TODO detect source in a more
      // robust way?
      frames++;

      // converting to (double) seconds (we shouldn't need the imageDelay anymore)
      double sourcets = _timestamp.getTimeMS() - imageDelay;
      double csTimeStampSeconds = sourcets / 1000.0;//(sourcets - imageDelay) / 1000.0;

      if (cs != nullptr) {
        cs->updateInternalModel(csTimeStampSeconds);
      }

    }
    for (auto &son : _children.at(filter->getName())) {
      std::string sonName = son->getName();

      dependenciesSolved[sonName]++;
      if ((size_t)dependenciesSolved[sonName] ==
          _filters[sonName]->_dependencies.size()) {
        list.push_back(son);
      }
    }
  }

  // And sweep through topological order
  // Do no process a filter twice
  while (!list.empty()) {

    Filter *filter = list.front();
    filter->runStep();

    for (auto &son : _children.at(filter->getName())) {
      std::string sonName = son->getName();
      dependenciesSolved[sonName]++;
      if ((size_t)dependenciesSolved[sonName] ==
          _filters[sonName]->_dependencies.size()) {
        list.push_back(son);
      }
    }
    list.pop_front();
  }
}

void Pipeline::runStep() {
  step(Filter::UpdateType::forward);
}

void Pipeline::resolveDependencies() {
  if (_children.size() != 0) {
    return;
  }

  for (auto &it : _filters) {
    _children[it.first] = std::vector<Filter *>();
  }

  // Preparing children map
  for (auto &it : _filters) {
    for (auto &dep : it.second->_dependencies) {
      _children[dep].push_back(it.second);
    }
  }

  // Checking consistency, is there
  for (auto &pair : _children) {
    const std::string &father = pair.first;
    try {
      _filters.at(father);
    } catch (const std::out_of_range &exc) {
      std::ostringstream oss;
      oss << "Unknown dependency '" << father
          << "', required from the following filters: ( ";
      for (size_t index = 0; index < pair.second.size(); index++) {
        oss << "'" << pair.second[index]->getName() << "'";
        if (index != pair.second.size() - 1)
          oss << ",";
      }
      oss << ")";
      throw std::runtime_error(oss.str());
    }
  }
}

Json::Value Pipeline::toJson() const {
  Json::Value v;
  for (auto &f : _filters) {
    v.append(f.second->toJson());
  }
  return v;
}

void Pipeline::addFiltersFromJson(const Json::Value & v, const std::string & dir_name) {
  Filters::FilterFactory ff;
  std::vector<std::unique_ptr<Filter>> result;
  if (v.isArray()) {
    for (Json::ArrayIndex idx = 0; idx < v.size(); idx++) {
      add(ff.build(v[idx], dir_name).release());
    }
  } else if (v.isObject()) {
    if (!v.isMember("filters") && ! v.isMember("paths")) {
      throw JsonParsingError(DEBUG_INFO +
                             " pipeline should contain either 'filters' or 'paths'");
    }
    if (v.isMember("filters")) {
      if (!v["filters"].isArray()) {
        throw JsonParsingError(DEBUG_INFO + " expecting an array for 'filters'");
      }
      try {
        addFiltersFromJson(v["filters"], dir_name);
      } catch (const JsonParsingError & err) {
        throw JsonParsingError(std::string(err.what()) + " in 'filters'");
      }
    }
    std::vector<std::string> paths;
    rhoban_utils::tryReadVector(v, "paths", &paths);
    for (const std::string & path : paths) {
      std::string file_path = dir_name + path;
      std::string read_dir_path = rhoban_utils::getDirName(file_path);
      Json::Value path_value =  file2Json(file_path);
      try {
        std::cout << "adding from " << file_path << std::endl;
        addFiltersFromJson(path_value, read_dir_path);
      } catch (const JsonParsingError & err) {
        throw JsonParsingError(std::string(err.what()) + " in '" + file_path + "'");
      }
    }
  } else {
    throw rhoban_utils::JsonParsingError(DEBUG_INFO +
                                         " pipeline is not an array neither an object");
  }
}
                                                     

void Pipeline::fromJson(const Json::Value & v, const std::string & dir_name) {
  addFiltersFromJson(v, dir_name);
  std::cout << "There is now " << _filters.size() << " filters." << std::endl;
}

void Pipeline::setTimestamp(const ::rhoban_utils::TimeStamp &ts) {
  _timestamp = ts;
}
  
const rhoban_utils::TimeStamp& Pipeline::getTimestamp() const {
  return _timestamp;
}
  
}
