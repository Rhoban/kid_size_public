#include "Filters/Source/SourceVideoProtobuf.hpp"

#include "CameraState/CameraState.hpp"
#include "Filters/Pipeline.hpp"
#include <FrameSource/Exceptions.hpp>

#include "rhoban_utils/util.h"
#include "rhoban_utils/timing/time_stamp.h"

#include <opencv2/opencv.hpp>

#include <stdexcept>

using namespace hl_monitoring;

namespace Vision
{
namespace Filters
{
SourceVideoProtobuf::SourceVideoProtobuf()
  : Source("SourceVideoProtobuf"), startIndex(0), index(0), keepOnlyMovingFrames(false)
{
}

void SourceVideoProtobuf::openVideo()
{
  if (!video.open(videoPath))
  {
    throw std::runtime_error(DEBUG_INFO + "failed to open video '" + videoPath + "'");
  }
}

void SourceVideoProtobuf::loadMetaInformation()
{
  loadMetaInformation(cameraFromWorldPath, &cameraFromWorldMeta);
  loadMetaInformation(cameraFromSelfPath, &cameraFromSelfMeta);
  if (cameraFromWorldMeta.frames_size() != cameraFromSelfMeta.frames_size())
  {
    throw std::runtime_error(DEBUG_INFO + " size mismatch between cameraFromWorld and cameraFromSelf");
  }
}

void SourceVideoProtobuf::loadMetaInformation(const std::string& path, hl_monitoring::VideoMetaInformation* out)
{
  std::ifstream in(path, std::ios::binary);
  if (!in.good())
  {
    throw std::runtime_error(DEBUG_INFO + " failed to open file '" + path + "'");
  }
  if (!out->ParseFromIstream(&in))
  {
    throw std::runtime_error(DEBUG_INFO + " failed to read file '" + path + "'");
  }
}

Utils::CameraState* SourceVideoProtobuf::buildCameraState()
{
  std::cout << "Update index:" << index << std::endl;
  if (index < 0 || index >= cameraFromWorldMeta.frames_size())
  {
    throw std::runtime_error(DEBUG_INFO + " invalid index: " + std::to_string(index));
  }
  if (!cameraFromWorldMeta.has_camera_parameters())
  {
    throw std::runtime_error(DEBUG_INFO + " camera_parameters were not provided");
  }

  const hl_monitoring::Pose3D& camera_from_self = cameraFromSelfMeta.frames(index).pose();
  return new Utils::CameraState(cameraFromWorldMeta.camera_parameters(), cameraFromWorldMeta.frames(index),
                                camera_from_self);
}

void SourceVideoProtobuf::process()
{
  updateImg();
}

void SourceVideoProtobuf::updateImg()
{
  index = (int)video.get(cv::CAP_PROP_POS_FRAMES);
  if (index < 0)
  {
    throw Utils::StreamEndException(DEBUG_INFO + " start of stream");
  }
  if (keepOnlyMovingFrames)
  {
    const FrameEntry& frame_entry = cameraFromWorldMeta.frames(index);
    if (!frame_entry.has_status() || frame_entry.status() != FrameStatus::MOVING)
    {
      setIndex(index);
    }
  }
  video.read(img());
  if (img().empty())
  {
    throw Utils::StreamEndException(DEBUG_INFO + " end of stream");
  }
}

void SourceVideoProtobuf::fromJson(const Json::Value& v, const std::string& dir_name)
{
  Filter::fromJson(v, dir_name);
  rhoban_utils::tryRead(v, "startIndex", &startIndex);
  rhoban_utils::tryRead(v, "videoPath", &videoPath);
  rhoban_utils::tryRead(v, "cameraFromWorldPath", &cameraFromWorldPath);
  rhoban_utils::tryRead(v, "cameraFromSelfPath", &cameraFromSelfPath);
  rhoban_utils::tryRead(v, "keepOnlyMovingFrames", &keepOnlyMovingFrames);
  openVideo();
  loadMetaInformation();

  if (startIndex != 0)
  {
    setIndex(startIndex - 1);
    updateImg();
  }
}

Json::Value SourceVideoProtobuf::toJson() const
{
  Json::Value v = Filter::toJson();
  v["startIndex"] = startIndex;
  v["videoPath"] = videoPath;
  v["cameraFromWorldPath"] = cameraFromWorldPath;
  v["cameraFromSelfPath"] = cameraFromSelfPath;
  v["keepOnlyMovingFrames"] = keepOnlyMovingFrames;
  return v;
}

std::string SourceVideoProtobuf::getClassName() const
{
  return "SourceVideoProtobuf";
}

int SourceVideoProtobuf::expectedDependencies() const
{
  return 0;
}

Source::Type SourceVideoProtobuf::getType() const
{
  return Type::Custom;
}

int SourceVideoProtobuf::getIndex() const
{
  return index;
}
int SourceVideoProtobuf::getNbFrames() const
{
  return (int)video.get(cv::CAP_PROP_FRAME_COUNT);
}

void SourceVideoProtobuf::setIndex(int target_index)
{
  int nb_frames = getNbFrames();
  while (keepOnlyMovingFrames && target_index < nb_frames)
  {
    const FrameEntry& frame_entry = cameraFromWorldMeta.frames(target_index);
    if (frame_entry.has_status() && frame_entry.status() == FrameStatus::MOVING)
    {
      break;
    }
    target_index++;
  }
  if (!video.set(cv::CAP_PROP_POS_FRAMES, target_index))
  {
    throw std::logic_error(DEBUG_INFO + " failed to set index at position " + std::to_string(index));
  }
}

bool SourceVideoProtobuf::isValid() const
{
  return video.isOpened();
}

void SourceVideoProtobuf::previous()
{
  setIndex(index - 1);
  updateImg();
}

void SourceVideoProtobuf::update()
{
  setIndex(index);
  updateImg();
}

}  // namespace Filters

}  // namespace Vision
