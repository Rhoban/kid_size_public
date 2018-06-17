#pragma once

#include <string>
#include <vector>
#include <unordered_set>
#include <thread>
#include <iostream>
#include <fstream>
#include <opencv2/core/core.hpp>
#include "Utils/ParamsContainer.hpp"
#include "Utils/SafePtr.hpp"

#include "rhoban_utils/serialization/json_serializable.h"

#include "rhoban_geometry/circle.h"

namespace Vision {
namespace Utils {
class CameraState;
}

class Pipeline;


enum ParameterType { DEFAULT, OUTPUT, INPUT, PARAM };

/**
 * Structure for filter parameter with
 * minimun and maximum bounds
 */
template <class T> struct Parameter {
  T value;
  T min;
  T max;
  ParameterType type;

  Parameter() {}

  Parameter(T value, T min, T max, ParameterType t = ParameterType::DEFAULT)
      : value(value), min(min), max(max), type(t) {}

  Parameter(T value) : value(value) {}

  Parameter<T> &operator=(const T &val) {
    value = val;
    return *this;
  }

  operator T &() { return value; }

  bool exportToRhIO() {
    switch (type) {
    case DEFAULT:
    case OUTPUT:
      return true;
    default:
      return false;
    }
  }

  bool importFromRhIO() {
    switch (type) {
    case DEFAULT:
    case PARAM:
    case INPUT:
      return true;
    default:
      return false;
    }
  }
};

template <class T>
inline std::ostream &operator<<(std::ostream &os, const Parameter<T> &p) {
  os << p.value << " (" << p.min << "-" << p.max << ")";
  return os;
}

/**
 * Typedef for Parameter int and float
 */
typedef Parameter<int> ParamInt;
typedef Parameter<float> ParamFloat;

/**
 * Filter
 *
 * Base class for image
 * processing step in the
 * pipeline (acyclic directed graph)
 */
class Filter : public rhoban_utils::JsonSerializable {
public:
  enum UpdateType { backward, forward, steady };

  /// Is the image displayed using openCV
  bool display;

  /**
   * Typedef for Parameter Container and
   * dependency container
   */
  typedef ParamsContainer<bool, int, long, float, double, ParamInt, ParamFloat,
                          std::vector<double>> Parameters;
  typedef std::vector<std::string> Dependencies;

  /**
   * Initialization with Filter name and
   * others needed filter names
   */
  Filter(const std::string &name) : Filter(name, {}) {}

  /**
   * Initialization with Filter name and
   * others needed filter names
   */
  Filter(const std::string &name, const Dependencies &dependencies);

  /**
   * Virtual destructor
   */
  virtual ~Filter();

  /**
   * Return Filter name
   */
  const std::string &getName() const;

  /**
   * Return output image
   */
  SafePtr<const cv::Mat> getImg() const;

  /**
   * Access to Filter Parameters
   */
  SafePtr<const Parameters> params() const;
  SafePtr<Parameters> params();

  Utils::CameraState &getCS();

  Pipeline *getPipeline();

  /**
   * Do a single processing step and return
   */
  void runStep(UpdateType updateType = UpdateType::forward);

  /// Initialize the display window
  void initWindow();

  // Json stuff
  virtual void fromJson(const Json::Value & v, const std::string & dir_name);
  virtual Json::Value toJson() const;
  virtual std::string getClassName() const { return "Filter"; }
  virtual int expectedDependencies() const { return 1; }

  const std::vector<std::pair<float, cv::RotatedRect>> &getRois() const;
  const std::vector<std::pair<float, rhoban_geometry::Circle>> &getCircleRois() const;
  void addRoi(float quality, cv::RotatedRect _rect);
  void addRoi(float quality, rhoban_geometry::Circle _circle);
  void clearRois();

  void addDependency(const std::string &name);

  static bool GPU_ON;

protected:
  // If an image is shown by a filter, then its name should be here
  static std::vector<std::string> listOfPresentWindows;

  /**
   * Set the parameters of the appropriated filter
   */
  virtual void setParameters() {}

  // In both case, path must end by /
  virtual void initRhIO(const std::string &path);
  virtual void importFromRhIO(const std::string &path);
  virtual void publishToRhIO(const std::string &path);


  /**
   * Return the Filter registered
   * as dependency by its name
   */
  const Filter &getDependency(const std::string &name) const;

  /**
   * Return the first registered dependency
   * if it exists or throw logical_error
   */
  const Filter &getDependency() const;

  /**
   * Access to output image buffer avalable for
   * writting
   * (double buffering)
   */
  cv::Mat &img();

  /// Access to the cached Image 
  cv::Mat & cachedImg();

  /**
   * Filter implementation called by run (switch to next image)
   */
  virtual void process(){};

  /**
   * Filter implementation called by run (switch to next image)
   */
  virtual void update();

  /**
   * Return to the previous image if possible, throw an exception if previous is
   * not available
   */
  virtual void previous();

  /**
   * Return the millisecond current timestamp
   */
  unsigned long now() const;


  /// Read only values of Parameters and not min or max
  template <typename T> void tryParametersUpdate(const Json::Value & v, const std::string & key) {
    std::map<std::string, T> parameters;
    rhoban_utils::tryReadMap(v, key, &parameters);
    for (const auto & pair : parameters) {
      params()->set<Parameter<T>>(pair.first) = pair.second;
    }
  }

  /// Write only values of Parameters and not min or max
  template <typename T> Json::Value parameters2Json() const {
    // Converting the ParameterMap to a map of basic type
    std::map<std::string, T> values;
    for (const auto & pair : params()->params<Parameter<T>>().getMap()) {
      values[pair.first] = pair.second.value;
    }
    return rhoban_utils::map2Json(values);
  }

  /**
   * The set of filter name needed
   * to compute output image
   */
  Dependencies _dependencies;

  /**
     * List of Regions of interest
     */
  std::vector<std::pair<float, cv::RotatedRect>>
      _rois; //<roi, quality> (.boundingRect() on a RotatedRect to get a Rect)
  std::vector<std::pair<float, rhoban_geometry::Circle>> _circleRois;

protected:

  /// Display the image
  void displayCurrent();

  /**
   * Filter name
   */
  std::string name;

  /**
   * Rhio name path
   */
  std::string rhio_path;

  /**
   * Current available image
   * (double buffering)
   */
  bool _availableImg;

  /**
   * Output computed image
   * (double buffering)
   */
  cv::Mat _img1;
  cv::Mat _img2;

  /**
   * Parameter container
   */
  Parameters _params;

  /**
   * Mutex lock
   */
  mutable std::mutex _lockParams;
  mutable std::mutex _lockImgs;

  /**
   * Access to Pipeline container instance
   * the Filter is registered into
   */
  Pipeline *_pipeline;

  /**
   * Access granted to Pipeline container
   */
  friend class Pipeline;

  /**
   * Says if we have a real ROI or the default image-wide one
   */
  bool _defaultRoi;

  /**
   * Which relative size of the image is used
   */
  double monitor_scale;

  /**
   * Is RhIO Initialized?
   */
  bool rhio_initialized;

  /// If the step of the  filter is longer than the limit, a warning is shown. [s]
  double warningExecutionTime;

  /**
   * Check and wait that dependencies are
   * fresh and updated
   */
  void waitDependencies() const;
};
}
