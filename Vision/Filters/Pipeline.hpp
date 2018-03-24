#pragma once

#include <map>
#include <vector>
#include <string>
#include <Filters/Filter.hpp>

#include "rhoban_utils/timing/time_stamp.h"

#include "rhoban_utils/serialization/json_serializable.h"

using namespace rhoban_utils;
namespace Vision {

namespace Utils {
class CameraState;
}

/**
 * Pipeline
 *
 * Groups Filter as acyclic oriented graph.
 * Handle Filter dependencies.
 */
class Pipeline : public rhoban_utils::JsonSerializable {
public:
  /**
   * Typedef for Filter container
   */
  typedef std::map<std::string, Filter *> FiltersMap;
  typedef std::vector<Filter *> RootFilters;
  typedef std::map<std::string, std::vector<Filter *>> FilterDependencies;
  typedef std::map<std::string, std::thread> FilterThread;

  /**
   * Initialization
   */
  Pipeline();

  /**
   * Destructor
   * Free all registered Filters
   * Do not free the registered CameraStateProvider
   */
  ~Pipeline();

  /**
   * Register a filter into the pipeline
   * The Filter deallocation is handle by the Pipeline
   */
  void add(Filter *filter);

  /**
   * Return the Filter with given name
   */
  const Filter &get(const std::string &name) const;
  Filter &get(const std::string &name);

  /**
   * Returns true if a filter called name is present, false otherwise
   */
  bool isFilterPresent(const std::string &name);

  /**
   * Deallocating the cameraState is up to the caller
   */

  Utils::CameraState *getCameraState();
  void setCameraState(Utils::CameraState *csInit);

  /**
   * Access to Filter container
   */
  const FiltersMap &filters() const;
  FiltersMap &filters();

  /**
   * Enable debuging for all Filters with
   * given level
   */
  void setDebugLevel(DebugLevel debugLevel = DebugLevel());

  /**
   * Write all Filters parameters to given
   * file (this file is overwriten)
   */
  void writeConfig(const std::string &path) const;

  /**
   * Read Filter parameters fron given file
   */
  void readConfig(const std::string &path);

  /**
   * Set and update the frequency of given
   * Filter name
   */
  void setFrequency(const std::string &name, Frequency::type freq);

  /**
   * Sweep throug all Filters a do a single
   * processing step
   */
  void runStep();

  /// Update all roots with the given updateType and then perform a processing
  /// step
  void step(Filter::UpdateType updateType);

  /**
   * Start all Filters processing
   * in a thread and return
   */
  void run();

  /// Set the timeStamp of the pipeline: not thread safe
  void setTimestamp(const ::rhoban_utils::TimeStamp & ts);
  /// Retrieve the timeStamp of the pipeline: not thread safe
  const ::rhoban_utils::TimeStamp& getTimestamp() const;

  // Json stuff
  virtual void fromJson(const Json::Value & v, const std::string & dir_name);
  virtual Json::Value toJson() const;
  virtual std::string getClassName() const { return "Pipeline"; }

  double imageDelay = 0.0;

  int frames = 0;

private:
  Utils::CameraState *cs;

  /**
   * Filter container
   */
  FiltersMap _filters;

  /**
   * Container for no dependency
   * Filters
   */
  RootFilters _rootFilters;

  /**
   * Container for childrens
   * dependencies
   */
  FilterDependencies _children;

  /**
   * Filter thread container
   */
  FilterThread _filterThread;

  /**
   * Timestamp of the current pipeline execution (usually updated by the source filter)
   */
  ::rhoban_utils::TimeStamp _timestamp;
  
  /**
   * Resolve dependencies for all Filters
   * Compute inversed dependency link
   */
  void resolveDependencies();

  /**
   * Resolve frequencies for
   * all filters
   */
  void resolveFrequencies();
};
}
