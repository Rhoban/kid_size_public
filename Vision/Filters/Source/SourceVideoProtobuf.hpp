#pragma once

#include "Filters/Source/Source.hpp"
#include "FrameSource/ImageSequence.hpp"

#include "video.pb.h"

#include <opencv2/videoio.hpp>

namespace Vision {
namespace Filters {

/**
 * SourceLogs
 *
 * This filter provide images based on the following elements:
 * - A video provides the images
 * - A protobuf file provides timestamps, and camera states
 */
class SourceVideoProtobuf : public Source {
public:
  SourceVideoProtobuf();

  // JSON stuff
  void fromJson(const Json::Value & v, const std::string & dir_name) override;
  Json::Value toJson() const override;
  std::string getClassName() const override;
  int expectedDependencies() const override;

  virtual Type getType() const override;

  int getIndex() const;
  int getNbFrames() const;
  void setIndex(int index);

  bool isValid() const;

  void update() override;
  void previous() override;
  void updateImg();

protected:
  /**
   * @Inherit
   */
  virtual void process() override;

  /// Open the video stream
  void openVideo();

  /// Load the MetaInformation
  void loadMetaInformation();

  Utils::CameraState * buildCameraState() override;

private:
  int startIndex;
  /// Path to the video including extension
  std::string videoPath;
  /// Path to the protobuf file containing the camera data
  std::string metaInformationPath;

  /// Index of the current image (0-based)
  int index;

  cv::VideoCapture video;

  rhoban_vision_proto::VideoMetaInformation videoMetaInformation;
};
}
}
