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

  /// Register a filter into the pipeline
  /// Ownership of 'filter' is given to the Pipeline
  void add(Filter *filter);

  /// Register all the filters from the provided vector
  /// Throws logic_error if a "null" filter is found or if there is a duplicated name
  void add(std::vector<std::unique_ptr<Filter>> * filters);

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

  /// Read a vector of filters from a Json value and add them to pipeline
  /// 1st format: value is an array of filters
  /// 2nd format: { "filters" : [f1,f2,...], "paths" : [relPath1,relPath2,...]}
  /// In second format, each path contains a list of filters
  void addFiltersFromJson(const Json::Value & v, const std::string & dir_name);

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

  /// Timestamp of the current pipeline execution (usually updated by the source filter)
  rhoban_utils::TimeStamp _timestamp;
  
  /// Resolve dependencies for all Filters Compute inversed dependency link
  void resolveDependencies();
};
}
