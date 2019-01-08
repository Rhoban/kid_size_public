#include "Filters/Basics/MovieRecorder.hpp"

#include <rhoban_utils/logging/logger.h>
#include <rhoban_utils/timing/time_stamp.h>

static rhoban_utils::Logger logger("MovieRecorder");

namespace Vision {
namespace Filters {

MovieRecorder::MovieRecorder() :
  Filter("MovieRecorder"),
  wasEnabled(false)
{
}

std::string MovieRecorder::getClassName() const
{
  return "MovieRecorder";
}

void MovieRecorder::setParameters() {
  enabled = ParamInt(0, 0, 1);
  framerate = ParamInt(30, 0, 100.0);
  params()->define<ParamInt>("enabled", &enabled);
  params()->define<ParamFloat>("framerate", &framerate);
}

void MovieRecorder::startStream(const cv::Size & size)
{
  std::string filename = rhoban_utils::getFormattedTime() + ".avi";
  bool useColor = true;//TODO: add as parameter
  videoWriter.open(filename, cv::VideoWriter::fourcc('X','V','I','D'), framerate, size, useColor);
//  videoWriter.set(cv::VIDEOWRITER_PROP_QUALITY, 100);
  if (!videoWriter.isOpened()) {
    logger.error("Failed to open video");
  }
}

void MovieRecorder::closeStream()
{
  videoWriter.release();
}

void MovieRecorder::process() {
  // Output is similar to input (for debug purpose)
  img() = *(getDependency().getImg());

  if (enabled) {
    if (!wasEnabled) {
      startStream(img().size());
    }
    videoWriter.write(img());
  } else if (wasEnabled) {
    closeStream();
  }

  wasEnabled = enabled;
}

}
}
