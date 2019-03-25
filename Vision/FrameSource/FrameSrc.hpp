#ifndef VISION_FRAMESRC_HPP
#define VISION_FRAMESRC_HPP

#include <string>
#include <opencv2/core/core.hpp>

#include "rhoban_utils/timing/time_stamp.h"

namespace Vision
{
/**
 * FrameSrc
 *
 * Base class interface
 * for Frame Source
 */
class FrameSrc
{
public:
  /**
   * Virtual destrutor
   */
  virtual ~FrameSrc()
  {
  }

  /**
   * Return true if the device
   * is successfully openned
   */
  virtual bool isOpened() const = 0;

  /**
   * Open and initialize video capture from
   * given source file or directory
   * Throw std::runtime_error on open error
   */
  virtual void open(const std::string& source) = 0;

  /**
   * Grab and return a new frame in RGB color format
   * Set the timestamp with the appropriate value
   * Throw std::runtime_error on grab error
   */
  // ugh
  virtual cv::Mat grabFrame(::rhoban_utils::TimeStamp& newTS, bool* lastCall) = 0;

  /**
   * Close and stop video capture
   * Throw std::runtime_error on grab error
   */
  virtual void close() = 0;
};
}  // namespace Vision

#endif
