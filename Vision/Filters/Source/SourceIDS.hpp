#include "Filters/Source/Source.hpp"

#include "rhoban_utils/timing/time_stamp.h"

#include <ueye.h>

#include <condition_variable>

namespace Vision {
namespace Filters {

/**
 * SourceIDS
 */
class SourceIDS : public Source {
public:
  /**
   * Do not open camera, simply create the Filter
   */
  SourceIDS();

  /**
   * Close capture device if required
   */
  virtual ~SourceIDS();

  // JSON stuff
  virtual void fromJson(const Json::Value & v, const std::string & dir_name) override;
  virtual Json::Value toJson() const override;

  virtual std::string getClassName() const override;
  virtual int expectedDependencies() const override;

  virtual Type getType() const override;

protected:
  virtual void setParameters() override;

  virtual void process() override;

  /**
   * Background thread which copies the content of the image received and
   * request information on the images
   *
   * Before launching it, camera should be connected and capture started
   */
  void backgroundProcess();

  /**
   * Is connection to the camera currently active?
   */
  bool isConnected();

  /**
   * Initialize the connection with the camera
   */
  void connect();

  /**
   * Initialize the connection with the camera
   */
  void disconnect();

  /**
   * Connect to camera and start capture
   */
  void startCamera();

  /**
   * End capture, close connection to camera and free the buffers if required
   * WARNING: does not support the case where buffers have been locked
   */
  void endCamera();

  /**
   * Retrieve the list of supported formats by the camera and stores it
   */
  void updateSupportedFormats();

  /**
   * Throws a IDSException if format is not allowed
   */
  void setFormat(int32_t format);

  /**
   * Update the format of the camera
   */
  void updateImageSettings();

  /**
   * Allocate memory buffers for the images
   */
  void allocateBuffers();

  /**
   * Set the exposure time in [ms]
   */
  void setExposure(double time);

  /**
   * Return current exposure time in [ms]
   */
  double getExposure();

protected:
  /**
   * Retrieve image from last buffer and related meta informations
   */
  void updateImage();
  
  /**
   * Update monitoring variables to RhIO
   */
  void updateRhIO();

private:
  /**
   * Contains an image associated with it's metadata
   */
  struct ImageEntry {
    cv::Mat img;
    UEYEIMAGEINFO image_info;
    /**
     * Date of reception of the signal IS_SET_EVENT_FRAME
     */
    rhoban_utils::TimeStamp ts;
  };
  
  /**
   * IDS camera
   */
  HIDS camera;

  /**
   * Is connected 
   */
  bool is_connected;

  /**
   * Is the image stream started
   */
  bool is_capturing;

  /**
   * The image currently used by the filter (foreground)
   */
  std::unique_ptr<ImageEntry> fg_entry;

  /**
   * The image currently used to store the background information
   */
  std::unique_ptr<ImageEntry> bg_entry;

  /**
   * Ensures the background thread is not overwritting an image currently in used
   */
  std::mutex bg_mutex;

  /**
   * Allows the 'process' thread to wait until a new image has been pushed
   */
  std::condition_variable bg_cond;

  /**
   * List of supported formats
   */
  std::vector<IMAGE_FORMAT_INFO> supported_formats;

  /**
   * Size of the image provided by the camera
   */
  cv::Size img_size;

  /**
   * Format of image used 
   */
  ParamInt format_id;

  /**
   * Exposure time in ms
   */
  ParamFloat exposure;

  /**
   * Number of requested frames per second
   */
  ParamFloat frame_rate;

  // Not tested, but should work

  /**
   * Return the success ratio of retrieving frames
   */
  double getSuccessRatio();

  /**
   * Number of success when getting frames
   */
  int nb_retrieve_success;

  /**
   * Number of failures while getting frames
   * - Currently failures are throwing exceptions and values is not incremented
   */
  int nb_retrieve_failures;
};
}
}
