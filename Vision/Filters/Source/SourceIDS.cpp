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

#define CHECK_CODE(code, msg) \
  if (code != IS_SUCCESS) { throw IDSException(DEBUG_INFO + msg + ": code " + std::to_string(code));}

using ::rhoban_utils::Benchmark;
using ::rhoban_utils::TimeStamp;
using Vision::Utils::IDSException;
using namespace std;

static rhoban_utils::Logger logger("SourceIDS");

namespace Vision {
namespace Filters {
  
SourceIDS::SourceIDS()
    : Source("SourceIDS"), is_capturing(false), nb_retrieve_success(0),
      nb_retrieve_failures(0) {
  is_connected = false;
}

SourceIDS::~SourceIDS() {}

void SourceIDS::fromJson(const Json::Value & v, const std::string & dir_name) {
  Filter::fromJson(v, dir_name);
}

Json::Value SourceIDS::toJson() const {
  return Filter::toJson();
}

string SourceIDS::getClassName() const {
  return "SourceIDS";
}

int SourceIDS::expectedDependencies() const {
  return 0;
}

Source::Type SourceIDS::getType() const {
  return Type::Online;
}

void SourceIDS::setParameters() {
  format_id = ParamInt(1,0,100);
  exposure = ParamFloat(2.0,0.01,100);
  frame_rate = ParamFloat(30.0,0.01,100);

  params()->define<ParamInt>("formatId", &format_id);
  params()->define<ParamFloat>("exposure", &exposure);
  params()->define<ParamFloat>("frameRate", &frame_rate);
}

void SourceIDS::process() {
  // If capture is not activated yet, start camera
  if (!is_capturing) {
    startCamera();
  }

  // TODO: apply parameters changes (detect them or overwrite anyway?)
  // - FormatId requires 
  setExposure(exposure);
  
  // Grab frame from camera
  updateImage();

  // TODO improve mechanism for TimeStamps:
  // - Require synchronization between camera clock and steady clock
  //   - Through software (unclear if is_DeviceFeature can do it through software)
  //   - Through hardware (using GPIO -> major changes required)
  getPipeline()->setTimestamp(fg_entry->ts);

  updateRhIO();
}

void SourceIDS::backgroundProcess() {
  if (!is_capturing) {
    throw IDSException(DEBUG_INFO + "capture was not started");
  }
  while (is_capturing) {
    int32_t ret_code;
    std::unique_ptr<ImageEntry> new_img(new ImageEntry);
    // Getting id of next buffer to be written
    int32_t buffer_id;
    char * buffer;
    ret_code = is_GetActSeqBuf(camera, &buffer_id, &buffer, NULL);
    CHECK_CODE(ret_code, "failed to get next buffer");
    // Waiting for a frame
    double wait_time_ms = 100;
    ret_code = is_WaitEvent(camera, IS_SET_EVENT_FRAME, wait_time_ms);
    CHECK_CODE(ret_code, "failed to wait for event");
    new_img->ts = TimeStamp::now();
    // Locking memory buffer
    ret_code = is_LockSeqBuf(camera, buffer_id, NULL);
    CHECK_CODE(ret_code, "failed to lock seq");
    // Retrieving image_info
    ret_code = is_GetImageInfo(camera, buffer_id,
                               &(new_img->image_info), sizeof(new_img->image_info));
    CHECK_CODE(ret_code, "failed to get image information");
    // Copying data to avoid overwrite
    cv::Mat tmp(img_size, CV_8UC3, buffer);
    tmp.copyTo(new_img->img);
    // Unlocking memory buffer
    ret_code = is_UnlockSeqBuf(camera, buffer_id, NULL);
    CHECK_CODE(ret_code, "failed to unlock seq");
    // Pushing data into memory and notifying eventual waiter (thread_safe)
    {
      std::unique_lock<std::mutex> lock(mutex);
      bg_entry = std::move(new_img);
      bg_cond.notify_all();
    }
    nb_retrieve_success++;
  }
}


bool SourceIDS::isConnected() {
  return is_connected;
}

void SourceIDS::connect() {
  int32_t ret = is_InitCamera(&camera, NULL);
  CHECK_CODE(ret, "failed to open camera");
  is_connected = true;
}

void SourceIDS::disconnect() {
  int32_t ret = is_ExitCamera(camera);
  CHECK_CODE(ret, "failed to close camera");
  is_connected = false;
}

void SourceIDS::startCamera() {
  // If connected, disconnect
  if (isConnected()) {
    disconnect();
  }
  connect();
  updateSupportedFormats();
  // Set appropriate mode and size
  updateImageSettings();
  // Start capture

  // TODO: handle potential errors on starting capture with a retry
  int32_t ret_code;
  ret_code = is_CaptureVideo(camera, IS_WAIT);
  CHECK_CODE(ret_code, "failed to start capture");
  is_capturing = true;
  // TODO Update internal properties (auto white balance etc...)
  setExposure(exposure);
}

void SourceIDS::endCamera() {
  int32_t ret_code;
  if (is_capturing) {
    ret_code = is_StopLiveVideo(camera, IS_WAIT);
    CHECK_CODE(ret_code, "Failed to stop capture");
    is_capturing = false;
  }
  ret_code = is_ClearSequence(camera);
  CHECK_CODE(ret_code, "Failed to clear buffers");
  //TODO: freeImageMem?
  if (isConnected()) {
    ret_code = is_ExitCamera(camera);
    CHECK_CODE(ret_code, "Failed to exit camera");
  }
}

void SourceIDS::updateSupportedFormats() {
  // Getting number of supported formats
  uint32_t entries;
  int32_t ret_code;
  ret_code = is_ImageFormat(camera, IMGFRMT_CMD_GET_NUM_ENTRIES, &entries, sizeof(entries));
  // Retrieving formats
  char formats[sizeof(IMAGE_FORMAT_LIST) + (entries-1)*sizeof(IMAGE_FORMAT_INFO)];
  IMAGE_FORMAT_LIST *formatList = (IMAGE_FORMAT_LIST*)formats;
  formatList->nNumListElements = entries;
  formatList->nSizeOfListEntry = sizeof(IMAGE_FORMAT_INFO);
  ret_code = is_ImageFormat(camera, IMGFRMT_CMD_GET_LIST, formats, sizeof(formats));
  CHECK_CODE(ret_code, "Failed to list image formats");
  for (uint32_t k=0; k<entries; k++) {
    supported_formats.push_back(formatList->FormatInfo[k]);
  }
}

void SourceIDS::setFormat(int32_t format_id) {
  for (IMAGE_FORMAT_INFO & info : supported_formats) {
    if (info.nFormatID == format_id) {
      int32_t ret_code;
      ret_code = is_ImageFormat(camera, IMGFRMT_CMD_SET_FORMAT,
                                &info.nFormatID, sizeof(info.nFormatID));
      CHECK_CODE(ret_code, "failed to set format");
      img_size = cv::Size(info.nWidth, info.nHeight);
      return;
    }
  }
  throw IDSException(DEBUG_INFO + "cannot find format with id: " + std::to_string(format_id));
}

void SourceIDS::updateImageSettings() {
  setFormat(format_id);
  //TODO: how is binning applied?
  //TODO: Simply print result of the format
  // Setting color mode to YCbCr
  int32_t ret_code;
  ret_code = is_SetColorMode(camera, IS_CM_CBYCRY_PACKED);
  CHECK_CODE(ret_code, "failed to set color_mode");
  //TODO: try IS_CM_JPEG
}

void SourceIDS::allocateBuffers() {
  int32_t ret;
  ret = is_ClearSequence(camera);
  CHECK_CODE(ret, "failed to clear sequence");
  // Setting the ring buffer for images for the camera
  int nb_buffers = 3;//TODO: set as parameter
  for (int k=0; k<nb_buffers; k++) {
    char *mem;
    int mem_id;
    int32_t bits_per_pixel = 16;// YCbCr -> 16 bits per pixel
    is_AllocImageMem(camera, img_size.width, img_size.height, bits_per_pixel, &mem, &mem_id);
    is_AddToSequence(camera, mem, mem_id);
  }
}

void SourceIDS::setExposure(double time) {
  int32_t ret;
  ret = is_Exposure(camera, IS_EXPOSURE_CMD_SET_EXPOSURE, &time, sizeof(double));
  CHECK_CODE(ret, "failed to set exposure time");
}

double SourceIDS::getExposure() {
  int32_t ret;
  double time;
  ret = is_Exposure(camera, IS_EXPOSURE_CMD_GET_EXPOSURE, &time, sizeof(double));
  CHECK_CODE(ret, "failed to get exposure time");
  return time;
}

void SourceIDS::updateImage() {
  std::unique_lock<std::mutex> lock(bg_mutex);
  while (!bg_entry) {
    bg_cond.wait(lock);
  }
  fg_entry = std::move(bg_entry);
}

void SourceIDS::updateRhIO() {
  std::string filter_path = rhio_path + getName();

  RhIO::IONode &monitoring_node = RhIO::Root.child(filter_path + "/monitoring");
  monitoring_node.setInt("success", nb_retrieve_success);
  monitoring_node.setInt("failures", nb_retrieve_failures);
  monitoring_node.setFloat("ratio", getSuccessRatio());
}

double SourceIDS::getSuccessRatio() {
  int nb_retrievals = nb_retrieve_success + nb_retrieve_failures;
  if (nb_retrievals == 0)
    return 1;
  return nb_retrieve_success / (double)nb_retrievals;
}

}
}

