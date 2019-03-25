#include "SourceIDS.hpp"

#include "Filters/Pipeline.hpp"
#include "Utils/IDSExceptions.hpp"

#include "RhIO.hpp"

#include "rhoban_utils/timing/benchmark.h"
#include <rhoban_utils/logging/logger.h>
#include <rhoban_utils/util.h>

#include <opencv2/highgui/highgui.hpp>

#include <thread>
#include <unistd.h>
#include <iomanip>

#define CHECK_CODE(code, msg)                                                                                          \
  if (code != IS_SUCCESS)                                                                                              \
  {                                                                                                                    \
    throw IDSException(DEBUG_INFO + msg + ": code " + std::to_string(code));                                           \
  }

using ::rhoban_utils::Benchmark;
using ::rhoban_utils::TimeStamp;
using Vision::Utils::IDSException;
using namespace std;

static rhoban_utils::Logger logger("SourceIDS");

namespace Vision
{
namespace Filters
{
SourceIDS::SourceIDS()
  : Source("SourceIDS")
  , is_capturing(false)
  , last_format_id(-1)
  , last_exposure(-1)
  , last_frame_rate(-1)
  , nb_retrieve_success(0)
  , nb_retrieve_failures(0)
{
  is_connected = false;
}

SourceIDS::~SourceIDS()
{
}

void SourceIDS::fromJson(const Json::Value& v, const std::string& dir_name)
{
  Filter::fromJson(v, dir_name);
  bindRhIO();
}

Json::Value SourceIDS::toJson() const
{
  return Filter::toJson();
}

string SourceIDS::getClassName() const
{
  return "SourceIDS";
}

int SourceIDS::expectedDependencies() const
{
  return 0;
}

Source::Type SourceIDS::getType() const
{
  return Type::Online;
}

void SourceIDS::setParameters()
{
  format_id = ParamInt(1, 0, 100);
  exposure = ParamFloat(2.0, 0.01, 100);
  frame_rate = ParamFloat(30.0, 0.01, 100);

  params()->define<ParamInt>("formatId", &format_id);
  params()->define<ParamFloat>("exposure", &exposure);
  params()->define<ParamFloat>("frameRate", &frame_rate);
}

void SourceIDS::process()
{
  // If capture is not activated yet, start camera
  if (!is_capturing)
  {
    startCamera();
  }

  if (requireImageSettingsUpdate())
  {
    stopCapture();
    updateImageSettings();
    startCapture();
  }

  updateLiveParameters();

  // Grab frame from camera
  updateImage();

  // TODO improve mechanism for TimeStamps:
  // - Require synchronization between camera clock and steady clock
  //   - Through software (unclear if is_DeviceFeature can do it through software)
  //   - Through hardware (using GPIO -> major changes required)
  getPipeline()->setTimestamp(fg_entry->ts);

  updateRhIO();
}

void SourceIDS::backgroundProcess()
{
  if (!is_capturing)
  {
    throw IDSException(DEBUG_INFO + "capture was not started");
  }
  int32_t ret_code;
  ret_code = is_EnableEvent(camera, IS_SET_EVENT_FRAME);
  CHECK_CODE(ret_code, "failed to set event frame");
  while (is_capturing)
  {
    try
    {
      std::unique_ptr<ImageEntry> new_img(new ImageEntry);
      // Getting id of next buffer to be written
      int32_t buffer_id;
      char* buffer;
      ret_code = is_GetActSeqBuf(camera, &buffer_id, &buffer, NULL);
      CHECK_CODE(ret_code, "failed to get next buffer");
      // Waiting for a frame
      double wait_time_ms = 5000;  // TODO: should depend on frame_rate
      ret_code = is_WaitEvent(camera, IS_SET_EVENT_FRAME, wait_time_ms);
      CHECK_CODE(ret_code, "failed to wait for event");
      new_img->ts = TimeStamp::now();
      // Locking memory buffer
      ret_code = is_LockSeqBuf(camera, buffer_id, NULL);
      CHECK_CODE(ret_code, "failed to lock seq");
      // Retrieving image_info
      ret_code = is_GetImageInfo(camera, buffer_id, &(new_img->image_info), sizeof(new_img->image_info));
      CHECK_CODE(ret_code, "failed to get image information");
      // Copying data to avoid overwrite
      cv::Mat tmp(img_size, CV_8UC3, buffer);
      new_img->img = tmp.clone();
      // Unlocking memory buffer
      ret_code = is_UnlockSeqBuf(camera, buffer_id, NULL);
      CHECK_CODE(ret_code, "failed to unlock seq");
      // Pushing data into memory and notifying eventual waiter (thread_safe)
      {
        std::unique_lock<std::mutex> lock(bg_mutex);
        bg_entry = std::move(new_img);
        bg_cond.notify_all();
      }
      nb_retrieve_success++;
    }
    catch (const IDSException& exc)
    {
      nb_retrieve_failures++;
      logger.error("Failed to retrieve image: %s", exc.what());
    }
  }
}

bool SourceIDS::isConnected()
{
  return is_connected;
}

void SourceIDS::connect()
{
  if (isConnected())
  {
    throw std::logic_error(DEBUG_INFO + "camera is already connected");
  }
  camera = 1;
  int32_t ret = is_InitCamera(&camera, NULL);
  CHECK_CODE(ret, "failed to open camera");
  is_connected = true;
  logger.log("Connection success");
}

void SourceIDS::disconnect()
{
  int32_t ret = is_ExitCamera(camera);
  CHECK_CODE(ret, "failed to close camera");
  is_connected = false;
  logger.log("Disconnection success");
}

void SourceIDS::startCamera()
{
  // If connected, disconnect
  if (isConnected())
  {
    disconnect();
  }
  connect();
  updateSupportedFormats();
  updateImageSettings();
  startCapture();
  updateLiveParameters();
}

void SourceIDS::endCamera()
{
  int32_t ret_code;
  if (is_capturing)
  {
    stopCapture();
  }
  if (isConnected())
  {
    ret_code = is_ExitCamera(camera);
    CHECK_CODE(ret_code, "Failed to exit camera");
  }
}

void SourceIDS::startCapture()
{
  allocateBuffers();

  // TODO: handle potential errors on starting capture with a retry
  int32_t ret_code;
  ret_code = is_CaptureVideo(camera, IS_WAIT);
  CHECK_CODE(ret_code, "failed to start capture");
  is_capturing = true;

  bg_thread = std::thread([this]() { this->backgroundProcess(); });
}

void SourceIDS::stopCapture()
{
  int32_t ret_code;
  ret_code = is_StopLiveVideo(camera, IS_WAIT);
  CHECK_CODE(ret_code, "Failed to stop capture");
  is_capturing = false;
  if (bg_thread.joinable())
  {
    bg_thread.join();
  }
  // TODO: freeImageMem?
  // TODO: handle case where buffers are locked
  ret_code = is_ClearSequence(camera);
  CHECK_CODE(ret_code, "Failed to clear buffers");
}

void SourceIDS::updateSupportedFormats()
{
  // Getting number of supported formats
  uint32_t entries;
  int32_t ret_code;
  ret_code = is_ImageFormat(camera, IMGFRMT_CMD_GET_NUM_ENTRIES, &entries, sizeof(entries));
  // Retrieving formats
  char formats[sizeof(IMAGE_FORMAT_LIST) + (entries - 1) * sizeof(IMAGE_FORMAT_INFO)];
  IMAGE_FORMAT_LIST* formatList = (IMAGE_FORMAT_LIST*)formats;
  formatList->nNumListElements = entries;
  formatList->nSizeOfListEntry = sizeof(IMAGE_FORMAT_INFO);
  ret_code = is_ImageFormat(camera, IMGFRMT_CMD_GET_LIST, formats, sizeof(formats));
  CHECK_CODE(ret_code, "Failed to list image formats");
  for (uint32_t k = 0; k < entries; k++)
  {
    supported_formats.push_back(formatList->FormatInfo[k]);
  }
  printSupportedFormats(&std::cout);
}

void SourceIDS::setFormat(int32_t format_id)
{
  if (last_format_id == format_id)
  {
    return;
  }
  for (IMAGE_FORMAT_INFO& info : supported_formats)
  {
    if (info.nFormatID == format_id)
    {
      int32_t ret_code;
      ret_code = is_ImageFormat(camera, IMGFRMT_CMD_SET_FORMAT, &info.nFormatID, sizeof(info.nFormatID));
      CHECK_CODE(ret_code, "failed to set format");
      img_size = cv::Size(info.nWidth, info.nHeight);
      last_format_id = format_id;
      return;
    }
  }
  throw IDSException(DEBUG_INFO + "cannot find format with id: " + std::to_string(format_id));
}

void SourceIDS::updateImageSettings()
{
  setFormat(format_id);
  // TODO: how is binning applied?
  // TODO: add choice of color_format
  int32_t ret_code;
  ret_code = is_SetColorMode(camera, IS_CM_BGR8_PACKED);
  CHECK_CODE(ret_code, "failed to set color_mode");
}

bool SourceIDS::requireImageSettingsUpdate()
{
  return last_format_id != format_id;
}

void SourceIDS::allocateBuffers()
{
  int32_t ret;
  ret = is_ClearSequence(camera);
  CHECK_CODE(ret, "failed to clear sequence");
  // Setting the ring buffer for images for the camera
  int nb_buffers = 3;  // TODO: set as parameter
  for (int k = 0; k < nb_buffers; k++)
  {
    char* mem;
    int mem_id;
    int32_t bits_per_pixel = 24;  // RGB8 -> 24 bits per pixel
    is_AllocImageMem(camera, img_size.width, img_size.height, bits_per_pixel, &mem, &mem_id);
    is_AddToSequence(camera, mem, mem_id);
  }
}

void SourceIDS::updateLiveParameters()
{
  // Note: order used for writing the parameters matters
  setFrameRate(frame_rate);
  setExposure(exposure);
}

void SourceIDS::setFrameRate(double fps)
{
  if (frame_rate == last_frame_rate)
  {
    return;
  }
  int32_t ret;
  double real_fps;
  ret = is_SetFrameRate(camera, fps, &real_fps);
  CHECK_CODE(ret, "failed to set fps to " + std::to_string(fps));
  std::string msg = "Fps set to " + std::to_string(real_fps) + " (required: " + std::to_string(fps) + ")";
  logger.log(msg.c_str());
  last_frame_rate = fps;
  // According to IDS Documentation, setting frame_rate might influence exposure
  // source: https://en.ids-imaging.com/manuals/uEye_SDK/EN/uEye_Manual_4.91.1/index.html
  last_exposure = -1;
}

void SourceIDS::setExposure(double time)
{
  if (time == last_exposure)
  {
    return;
  }
  int32_t ret;
  ret = is_Exposure(camera, IS_EXPOSURE_CMD_SET_EXPOSURE, &time, sizeof(double));
  CHECK_CODE(ret, "failed to set exposure time");
  logger.log("Set exposure to %lf ms", time);
  last_exposure = time;
}

double SourceIDS::getExposure()
{
  int32_t ret;
  double time;
  ret = is_Exposure(camera, IS_EXPOSURE_CMD_GET_EXPOSURE, &time, sizeof(double));
  CHECK_CODE(ret, "failed to get exposure time");
  return time;
}

void SourceIDS::updateImage()
{
  std::unique_lock<std::mutex> lock(bg_mutex);
  while (!bg_entry)
  {
    bg_cond.wait(lock);
  }
  fg_entry = std::move(bg_entry);
  img() = fg_entry->img;
}

void SourceIDS::bindRhIO()
{
  std::string filter_path = rhio_path + getName();

  // Declare monitoring variables if it has not been done yet
  std::string monitoring_path = filter_path + "/monitoring";
  if (!RhIO::Root.childExist(monitoring_path))
  {
    // Create monitoring node
    RhIO::Root.newChild(monitoring_path);
    RhIO::IONode& node = RhIO::Root.child(monitoring_path);
    // Create Value nodes
    node.newInt("success")->defaultValue(nb_retrieve_success);
    node.newInt("failures")->defaultValue(nb_retrieve_failures);
    node.newFloat("ratio")->defaultValue(getSuccessRatio());
  }
}

void SourceIDS::updateRhIO()
{
  std::string filter_path = rhio_path + getName();

  RhIO::IONode& monitoring_node = RhIO::Root.child(filter_path + "/monitoring");
  monitoring_node.setInt("success", nb_retrieve_success);
  monitoring_node.setInt("failures", nb_retrieve_failures);
  monitoring_node.setFloat("ratio", getSuccessRatio());
}

void SourceIDS::printSupportedFormats(std::ostream* out) const
{
  for (const IMAGE_FORMAT_INFO& info : supported_formats)
  {
    (*out) << info << std::endl;
  }
}

double SourceIDS::getSuccessRatio()
{
  int nb_retrievals = nb_retrieve_success + nb_retrieve_failures;
  if (nb_retrievals == 0)
    return 1;
  return nb_retrieve_success / (double)nb_retrievals;
}

std::ostream& operator<<(std::ostream& os, const IMAGE_FORMAT_INFO& format)
{
  os << "(id:" << format.nFormatID << ","
     << " size:" << format.nWidth << "x" << format.nHeight << ","
     << " aoiStart:(" << format.nX0 << "," << format.nY0 << "),"
     << " captureModes:" << format.nSupportedCaptureModes << ","
     << " binningMode:" << format.nBinningMode << ","
     << " subsamplingMode:" << format.nSubsamplingMode << ","
     << " formatName:" << format.strFormatName << ","
     << " scalingFactor:" << format.dSensorScalerFactor << ")";
  return os;
}

}  // namespace Filters
}  // namespace Vision
