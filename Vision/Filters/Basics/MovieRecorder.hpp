#pragma once

#include "Filters/Filter.hpp"

#include <opencv2/videoio.hpp>

namespace Vision {
namespace Filters {

/**
 * MovieRecorder
 *
 * Save movies to files
 */
class MovieRecorder : public Filter {
public:
  MovieRecorder();

  std::string getClassName() const override;

protected:
  /**
   * @Inherit
   */
  virtual void process() override;

  virtual void setParameters() override;

private:

  /// Automatically choose a name for the movie and open the stream
  void startStream(const cv::Size & size);

  void closeStream();
  
  /// While enabled, saves to a file 
  ParamInt enabled;

  /// Framerate used for video produced
  ParamFloat framerate;

  /// Status at last tick
  bool wasEnabled;

  cv::VideoWriter videoWriter;
};
}
}
