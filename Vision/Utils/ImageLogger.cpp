#include <iostream>
#include "ImageLogger.h"

#include <hl_communication/utils.h>
#include <rhoban_utils/util.h>

#include <opencv2/highgui/highgui.hpp>

using namespace hl_communication;

namespace Vision
{
namespace Utils
{
ImageLogger::ImageLogger(const std::string& logger_prefix, bool store_images_, int max_img)
  : logger_prefix(logger_prefix), store_images(store_images_), max_img(max_img)
{
}

bool ImageLogger::isActive() const
{
  return session_path != "";
}

void ImageLogger::pushEntry(const ImageLogger::Entry& cst_entry)
{
  // Start session if required
  if (!isActive())
  {
    initSession(cst_entry.cs);
  }
  // If too much images have been written, throw a SizeLimitException
  if (img_index >= max_img)
  {
    throw SizeLimitException(DEBUG_INFO + " max images reached");
  }
  // Duplicate img data (avoid corruption)
  Entry entry;
  entry.img = cst_entry.img.clone();
  entry.time_stamp = cst_entry.time_stamp;
  entry.cs = cst_entry.cs;
  // Store or write imaged depending on mode
  if (store_images)
  {
    entries_map[img_index] = entry;
  }
  else
  {
    writeEntry(img_index, entry);
  }
  img_index++;
}

void ImageLogger::endSession()
{
  if (entries_map.size() != 0)
  {
    for (const auto& pair : entries_map)
    {
      writeEntry(pair.first, pair.second);
    }
  }
  video_writer.release();
  // Can only be written in the end because delimited writing of messages is not supported in protobuf 3.0.0
  for (auto& entry : metadata)
  {
    std::string log_path = session_path + "/" + entry.first + ".pb";
    hl_communication::writeToFile(log_path, entry.second);
  }
  metadata.clear();
  session_path = "";
  entries_map.clear();
  img_index = 0;
}

void ImageLogger::initSession(const CameraState& cs, const std::string& session_local_path)
{
  if (session_local_path != "")
  {
    session_path = logger_prefix + "/" + session_local_path;
  }
  else
  {
    // Use a default name
    session_path = logger_prefix + "/" + rhoban_utils::getFormattedTime();
  }
  int err = system(("mkdir -p " + session_path).c_str());
  if (err != 0)
  {
    throw std::runtime_error(DEBUG_INFO + "Failed to create dir: '" + session_path + "'");
  }
  std::string filename = session_path + "/video.avi";
  double framerate = 30;
  bool use_color = true;
  video_writer.open(filename, cv::VideoWriter::fourcc('X', 'V', 'I', 'D'), framerate, cs.getImgSize(), use_color);
  if (!video_writer.isOpened())
  {
    throw std::runtime_error(DEBUG_INFO + "Failed to open video");
  }
  hl_communication::VideoMetaInformation meta_information;
  cs.exportHeader(&meta_information);
  meta_information.set_time_offset(rhoban_utils::getSteadyClockOffset());
  for (const std::string& log_name :
       { "camera_from_world", "camera_from_field", "camera_from_self", "camera_from_head_base" })
  {
    metadata[log_name] = meta_information;
  }
  // Writing Metadata file
  Json::Value log_metadata;
  log_metadata["robot"] = rhoban_utils::getHostName();
  rhoban_utils::writeJson(log_metadata, session_path + "/metadata.json");
  // Copying calibration.json file
  std::string cmd = "cp calibration.json " + session_path;
  int ret = system(cmd.c_str());
  if (ret != 0)
  {
    throw std::runtime_error(DEBUG_INFO + " failed to copy calibration.json file with code " + std::to_string(ret));
  }
}

const std::string& ImageLogger::getSessionPath()
{
  return session_path;
}

void ImageLogger::writeEntry(int idx, const Entry& e)
{
  // Writing image
  video_writer.write(e.img);
  // Adding entry_properties to metadata (cannot write in file before end of session)
  hl_communication::FrameEntry* entry = metadata["camera_from_world"].add_frames();
  e.cs.exportToProtobuf(entry);
  entry = metadata["camera_from_self"].add_frames();
  e.cs.exportToProtobuf(entry);
  setProtobufFromAffine(e.cs.worldToCamera * e.cs.selfToWorld, entry->mutable_pose());
  entry = metadata["camera_from_head_base"].add_frames();
  e.cs.exportToProtobuf(entry);
  setProtobufFromAffine(e.cs.cameraFromHeadBase, entry->mutable_pose());
  entry = metadata["camera_from_field"].add_frames();
  e.cs.exportToProtobuf(entry);
  if (e.cs.has_camera_field_transform)
  {
    setProtobufFromAffine(e.cs.camera_from_field, entry->mutable_pose());
  }
  else
  {
    entry->clear_pose();
  }
}

}  // namespace Utils
}  // namespace Vision
