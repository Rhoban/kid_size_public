#pragma once

#include "Filters/Source/Source.hpp"
#include "FrameSource/ImageSequence.hpp"

#include <hl_communication/camera.pb.h>

#include <opencv2/videoio.hpp>

namespace Vision
{
namespace Filters
{
/**
 * SourceLogs
 *
 * This filter provide images based on the following elements:
 * - A video provides the images
 * - A protobuf file provides timestamps, and camera states
 */
class SourceVideoProtobuf : public Source
{
public:
  SourceVideoProtobuf();

  // JSON stuff
  void fromJson(const Json::Value& v, const std::string& dir_name) override;
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
  void loadMetaInformation(const std::string& path, hl_communication::VideoMetaInformation* out);

  Utils::CameraState* buildCameraState() override;

private:
  int startIndex;
  /// Path to the video including extension
  std::string videoPath;
  /// Path to the protobuf file containing the camera data

  /// Index of the current image (0-based)
  int index;

  cv::VideoCapture video;

  /**
   * If enabled, static and shaking frames are skipped
   */
  bool keepOnlyMovingFrames;

  /**
   * Path access for cameraFromWorld
   */
  std::string cameraFromWorldPath;
  /**
   * Meta information for cameraFromWorld
   */
  hl_communication::VideoMetaInformation cameraFromWorldMeta;

  /**
   * Path access for cameraFromSelf
   */
  std::string cameraFromSelfPath;
  /**
   * Meta information for cameraFromSelf
   */
  hl_communication::VideoMetaInformation cameraFromSelfMeta;

  /**
   * Path access for cameraFromHeadBase
   */
  std::string cameraFromHeadBasePath;
  /**
   * Meta information for cameraFromHeadBase
   */
  hl_communication::VideoMetaInformation cameraFromHeadBaseMeta;
};
}  // namespace Filters
}  // namespace Vision
