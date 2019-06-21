#pragma once

#include "Filters/Filter.hpp"

#include <hl_communication/camera.pb.h>

#include <opencv2/videoio.hpp>

namespace Vision
{
namespace Filters
{
/**
 * MovieRecorder
 *
 * Save movies to files
 *
 * TODO: add option of adding padding of similar images when images are missing.
 */
class MovieRecorder : public Filter
{
public:
  MovieRecorder();
  virtual ~MovieRecorder();

  std::string getClassName() const override;

  void finish() override;

protected:
  /**
   * @Inherit
   */
  virtual void process() override;

  virtual void setParameters() override;

private:
  /// Automatically choose a name for the movie and open the stream
  void startStream(const cv::Size& size);

  /// Push entry to current stream
  void pushEntry();

  void closeStream();

  /// While enabled, saves to a file
  ParamInt enabled;

  /// Framerate used for video produced
  ParamFloat framerate;

  /// Status at last tick
  bool wasEnabled;

  cv::VideoWriter videoWriter;

  /// Name of the video currently written (without extension)
  std::string videoPath;

  hl_communication::VideoMetaInformation videoMetaInformation;
};
}  // namespace Filters
}  // namespace Vision
