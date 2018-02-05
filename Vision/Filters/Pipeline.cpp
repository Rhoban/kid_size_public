#include <stdexcept>
#include <list>
#include <functional>
#include <fstream>
#include "Filters/Pipeline.hpp"
#include <iostream>
#include "Filters/FilterFactory.hpp"
#include "CameraState/CameraState.hpp"

#include "Localisation/RobotBasis.hpp"
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
  // End Filters thread
  for (auto &it : _filters) {
    it.second->_stopThread = true;
  }
  for (auto &it : _filterThread) {
    it.second.join();
  }

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
  /*
    try{
    *(_filters.at(name));
    }
    catch(const std::out_of_range & e){
    return false;
    }
    return true;
  */

  if (_filters.find(name) == _filters.end()) {
    return false;
  } else {
    // if(_filters.at(name) )
    //     return true;
    // return false;
    return true;
  }
}

const Pipeline::FiltersMap &Pipeline::filters() const { return _filters; }
Pipeline::FiltersMap &Pipeline::filters() { return _filters; }

void Pipeline::enableDump(const std::string &path) {
  for (auto &it : _filters) {
    it.second->enableDump(path);
  }
}

void Pipeline::setDebugLevel(DebugLevel newLevel) {
  for (auto &it : _filters) {
    it.second->debugLevel = newLevel;
  }
}

void Pipeline::writeConfig(const std::string &path) const {
  std::ofstream file;
  file.open(path);

  if (!file.is_open()) {
    throw std::runtime_error("Pipeline unable to open/write config file " +
                             path);
  }

  for (const auto &it : _filters) {
    it.second->writeConfig(file);
  }
  file.close();
}

void Pipeline::readConfig(const std::string &path) {
  std::ifstream file;
  file.open(path);

  if (!file.is_open()) {
    throw std::runtime_error("Pipeline unable to open/read config file " +
                             path);
  }

  while (file.good()) {
    char filterName[128];
    char paramName[128];
    char valueStr[128];
    // Extract filter name
    file.get(filterName, 128, '.');
    if (!file.good())
      break;
    file.ignore();
    // Extract parameter name
    file.get(paramName, 128, ':');
    if (!file.good())
      break;
    file.ignore();
    if (file.peek() == ' ') {
      file.ignore();
    }
    // And string value
    file.get(valueStr, 128, '\n');
    if (!file.good())
      break;
    file.ignore();
    // Then call the associated filter to parse the value
    if (_filters.count(filterName) != 0) {
      _filters[filterName]->readConfig(paramName, valueStr);
    }
  }

  file.close();
}

void Pipeline::setFrequency(const std::string &name, Frequency::type freq) {
  if (freq == Frequency::Auto) {
    throw std::logic_error("Pipeline invalid set auto frequency");
  }

  get(name).setFrequency(freq);
  resolveFrequencies();
}

bool Pipeline::step(Filter::UpdateType updateType) {
  Benchmark::open("Resolve dependecies");
  resolveDependencies();
  Benchmark::close("Resolve dependecies");

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
    if (filter->terminatePipeline) {
      return false;
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

    if (filter->terminatePipeline) {
      std::cout << "TerminatePipeline in pipeline.cpp" << std::endl;
      return false;
    }
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

  return true;
}

void Pipeline::runStep() { step(Filter::UpdateType::forward); }

// Commented cos not used and scary to read
/*void Pipeline::run()
{
  resolveDependencies();
  resolveFrequencies();

  //Start all Filter in a thread
  for (auto& it : _filters) {
      _filterThread[it.first] = std::thread(
          std::bind(&Filter::run, it.second));
  }
  }*/

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

void Pipeline::resolveFrequencies() {
  bool isModif = true;
  while (isModif) {
    isModif = false;
    for (auto &it : _filters) {
      // Check that leaf filters are not Auto
      if (*(it.second->getFrequency()) == Frequency::Auto &&
          _children[it.first].size() == 0) {
        throw std::logic_error(
            "Pipeline invalid leaf filter with Auto frequency");
      }
      // Resolve Auto frequency by finding the maximum
      // frequency of children for all non leaf Filters
      if (_children[it.first].size() != 0) {
        Frequency::type max = Frequency::Stop;
        for (const auto &dep : _children[it.first]) {
          if (*(dep->getFrequency()) == Frequency::Auto) {
            max = Frequency::Auto;
            break;
          } else if (max < *(dep->getFrequency())) {
            max = *(dep->getFrequency());
          }
        }
        if (max != *(it.second->getFrequency())) {
          it.second->setFrequency(max);
          isModif = true;
        }
      }
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
void Pipeline::fromJson(const Json::Value & v, const std::string & dir_name) {
  Filters::FilterFactory ff;
  for (Json::ArrayIndex idx = 0; idx < v.size(); idx++) {
    add(ff.build(v[idx], dir_name).release());
  }
  std::cout << "There is now " << _filters.size() << " filters." << std::endl;
}

void Pipeline::setTimestamp(const ::rhoban_utils::TimeStamp &ts) {
  _timestamp = ts;
}
  
const ::rhoban_utils::TimeStamp& Pipeline::getTimestamp() const {
  return _timestamp;
}
  
}
