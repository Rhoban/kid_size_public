#include "Filters/Source/SourceVideoProtobuf.hpp"

#include "CameraState/CameraState.hpp"
#include "Filters/Pipeline.hpp"
#include <FrameSource/Exceptions.hpp>

#include "rhoban_utils/util.h"
#include "rhoban_utils/timing/time_stamp.h"

#include <opencv2/opencv.hpp>

#include <stdexcept>

namespace Vision
{
namespace Filters
{
SourceVideoProtobuf::SourceVideoProtobuf() : Source("SourceVideoProtobuf"), startIndex(0), index(0)
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
  std::ifstream in(metaInformationPath, std::ios::binary);
  if (!in.good())
  {
    throw std::runtime_error(DEBUG_INFO + " failed to open file '" + metaInformationPath + "'");
  }
  if (!videoMetaInformation.ParseFromIstream(&in))
  {
    throw std::runtime_error(DEBUG_INFO + " failed to read file '" + metaInformationPath + "'");
  }
}

Utils::CameraState* SourceVideoProtobuf::buildCameraState()
{
  if (index < 0 || index >= videoMetaInformation.frames_size())
  {
    throw std::runtime_error(DEBUG_INFO + " invalid index: " + std::to_string(index));
  }
  if (!videoMetaInformation.has_camera_parameters())
  {
    throw std::runtime_error(DEBUG_INFO + " camera_parameters were not provided");
  }
  return new Utils::CameraState(videoMetaInformation.camera_parameters(), videoMetaInformation.frames(index));
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
  rhoban_utils::tryRead(v, "metaInformationPath", &metaInformationPath);
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
  v["metaInformationPath"] = metaInformationPath;
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
