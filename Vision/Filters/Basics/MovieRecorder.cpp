#include "Filters/Basics/MovieRecorder.hpp"

#include "CameraState/CameraState.hpp"

#include <rhoban_utils/util.h>
#include <rhoban_utils/logging/logger.h>
#include <rhoban_utils/timing/time_stamp.h>

static rhoban_utils::Logger logger("MovieRecorder");

namespace Vision
{
namespace Filters
{
MovieRecorder::MovieRecorder() : Filter("MovieRecorder"), wasEnabled(false)
{
}

MovieRecorder::~MovieRecorder()
{
  closeStream();
}

std::string MovieRecorder::getClassName() const
{
  return "MovieRecorder";
}

void MovieRecorder::finish()
{
  closeStream();
}

void MovieRecorder::setParameters()
{
  enabled = ParamInt(0, 0, 1);
  framerate = ParamInt(30, 0, 100.0);
  params()->define<ParamInt>("enabled", &enabled);
  params()->define<ParamFloat>("framerate", &framerate);
}

void MovieRecorder::startStream(const cv::Size& size)
{
  if (videoPath != "")
  {
    throw std::logic_error(DEBUG_INFO + "A video is already being written to " + videoPath);
  }
  // videoPath = rhoban_utils::getFormattedTime();
  videoPath = "movie_recorder_output";
  std::string filename = videoPath + ".avi";
  bool useColor = true;  // TODO: add as parameter
  // TODO: add quality and fourcc as parameter
  videoWriter.open(filename, cv::VideoWriter::fourcc('X', 'V', 'I', 'D'), framerate, size, useColor);
  //  videoWriter.set(cv::VIDEOWRITER_PROP_QUALITY, 100);
  if (!videoWriter.isOpened())
  {
    throw std::runtime_error(DEBUG_INFO + "Failed to open video");
  }

  // Setting intrinsic parameters
  hl_communication::IntrinsicParameters* intrinsic = videoMetaInformation.mutable_camera_parameters();
  getCS().exportToProtobuf(intrinsic);
  if (intrinsic->img_width() != (size_t)size.width || intrinsic->img_height() != (size_t)size.height)
  {
    // TODO: rescale parameters if size is not the same
    throw std::logic_error(DEBUG_INFO + " size of provided image is not the same as camerastate size");
  }
}

void MovieRecorder::pushEntry()
{
  videoWriter.write(img());
  const Utils::CameraState& cs = getCS();
  hl_communication::FrameEntry* new_fe = videoMetaInformation.add_frames();
  cs.exportToProtobuf(new_fe);
}

void MovieRecorder::closeStream()
{
  videoWriter.release();

  std::string metadataPath = videoPath + "_metadata.pb";
  std::ofstream out(metadataPath);
  if (!out.good())
  {
    throw std::runtime_error(DEBUG_INFO + " failed to open '" + metadataPath + "'");
  }
  if (!videoMetaInformation.SerializeToOstream(&out))
  {
    throw std::runtime_error(DEBUG_INFO + " failed to write in '" + metadataPath + "'");
  }
  out.close();

  // Clearing existing meta informations
  videoMetaInformation.clear_camera_parameters();
  videoMetaInformation.clear_frames();

  videoPath.clear();
}

void MovieRecorder::process()
{
  // Output is similar to input (for debug purpose)
  img() = *(getDependency().getImg());

  if (enabled)
  {
    if (!wasEnabled)
    {
      startStream(img().size());
    }
    pushEntry();
  }
  else if (wasEnabled)
  {
    closeStream();
  }

  wasEnabled = enabled;
}

}  // namespace Filters
}  // namespace Vision
