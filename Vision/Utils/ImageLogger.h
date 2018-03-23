#pragma once

#include <rhoban_utils/timing/time_stamp.h>

#include <opencv2/core/core.hpp>

#include <fstream>
#include <map>
#include <stdexcept>

namespace Vision {
namespace Utils {

/// Handles logging of images and lowlevel data
///
/// This class uses the notion of 'logSession':
/// - A session is opened when the first image is pushed
/// - The session is closed manually
///
/// When a session is opened a folder with an 'unique' name is created (name is
/// based on system clock). Images are all written in this folder.
///
/// There are two different modes of logging:
/// 1. Images are stored and logged at the end of the session
/// 2. Images are written directly
class ImageLogger {
public:
  typedef std::pair<rhoban_utils::TimeStamp,cv::Mat> Entry;

  /// Create a Logger
  /// @param logger_prefix The prefix for all sessions using this ImageLogger
  /// @param store_images If true, images are stored and dumped at the end of the session
  /// @param max_images The maximal number of images allowed for a single log session
  ImageLogger(const std::string & logger_prefix, bool store_images, int max_images);

  /// Is the logger currently active?
  bool isActive() const;

  /// Start a session at "logger_prefix/session_local_path"
  /// Create the directory if necessary
  /// If session_local_path is an empty string, then generates a name based on current time
  void initSession(const std::string & session_local_path = "");

  /// Push the entry in the log, if there is no active session, open a new session
  /// @throws a SizeLimitException if the maximal number of images per session has been reached
  void pushEntry(const Entry & entry);

  /// Close current session and dump images if necessary
  void endSession();

  /// Return the path to current session (empty if no session is in progress)
  const std::string & getSessionPath();

  class SizeLimitException : public std::runtime_error {
  public:
    SizeLimitException(const std::string & what) : std::runtime_error(what) {}
  };

private:
  /// The prefix of the folder name containing the logs
  std::string logger_prefix;
  /// Are images written directly or stored in memory?
  /// This value should not be changed during a log
  bool store_images;
  /// The number of images written in the current session
  int img_index;
  /// The maximal number of images allowed for a single session
  int max_img;
  /// Images and associated time_stamps
  std::map<int,Entry> entries_map;
  /// Number of images allowed per log session
  /// Active session path
  std::string session_path;
  /// Logs also produce a file mapping images ids to timestamps
  std::ofstream description_file;

  /// Write a line in the csv file associating file names with timestamps and a
  /// png image for the given file
  void writeEntry(int idx, const Entry & e);
};


}}
