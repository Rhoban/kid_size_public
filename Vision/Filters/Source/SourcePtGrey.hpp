#include "Filters/Source/Source.hpp"

#include "rhoban_utils/timing/time_stamp.h"

#include <flycapture/FlyCapture2.h>

/// How to configure the camera to have a static ip:
/// https://www.ptgrey.com/KB/10933

/// Headers usually found in /usr/include/flycapture

namespace Vision
{
namespace Filters
{
/**
 * SourcePtGrey
 */
class SourcePtGrey : public Source
{
public:
  /**
   * Do not open camera
   */
  SourcePtGrey();

  /**
   * Close capture device
   */
  virtual ~SourcePtGrey();

  // JSON stuff
  FlyCapture2::Property propertyFromJson(const Json::Value& v, FlyCapture2::PropertyType type) const;
  virtual void fromJson(const Json::Value& v, const std::string& dir_name) override;
  virtual Json::Value toJson() const override;

  virtual std::string getClassName() const override;
  virtual int expectedDependencies() const override;

  virtual Type getType() const override;

protected:
  /**
   * @Inherit
   */
  virtual void process() override;

  /**
   * Connect to camera and start capture
   */
  void startCamera();

  /**
   * End capture and close connection to camera
   */
  void endCamera();

  /// Cut current connection if active, then start a new connection
  void reconnectCamera();

  /// Update the imaging mode, binning and PixelFormat of the camera
  void updateImageSettings();

  /// Update the packet size and the packet delay to reduce image
  /// inconsistencies
  /// NOTE: this has to be done while the camera is not capturing
  void updatePacketProperties();

  /**
   * Update the status of all known properties by reading the camera
   */
  void updateProperties();

  /**
   * Update the informations about all known properties by reading the camera
   */
  void updatePropertiesInformation();

  /// Debug readable info
  void dump(const FlyCapture2::Format7ImageSettings& image_settings, std::ostream& out);

  /// Debug readable info
  void dump(const FlyCapture2::GigEImageSettings& image_settings, std::ostream& out);

  /// Debug readable info
  void dump(const FlyCapture2::GigEImageSettingsInfo& image_settings_info, std::ostream& out);

  /**
   * Dump all the current properties on the provided stream
   */
  void dumpProperties(std::ostream& out);

  /**
   * Dump all the properties information on the provided stream
   */
  void dumpPropertiesInformation(std::ostream& out);

  /**
   * Estimates the delay between the camera internal clock and the local pc clock.
   */
  double measureTimestampDelta();

  /**
   * Correspondance between names and PropertyTypes
   * throws std::out_of_range if name is not registered
   */
  static FlyCapture2::PropertyType getPropertyType(const std::string& name);

  /**
   * Write the property to the given stream
   */
  static void writeProperty(const FlyCapture2::Property& property, std::ostream& out, const std::string& prefix = "");

  /**
   * Write the property to the given stream
   */
  static void writePropertyInformation(const FlyCapture2::PropertyInfo& property_info, std::ostream& out,
                                       const std::string& prefix = "");

protected:
  /// Bind properties and monitoring variables to RhIO
  void bindProperties();

  /// Import properties to 'wished properties'
  void importPropertiesFromRhIO();

  /// Update monitoring variables to RhIO
  void updateRhIO();

  /// Apply 'wished properties' to camera
  void applyWishedProperties();
  // TODO publishToRhIO

  /// Set timeout for grabFrames, necessary to ensure that vision does not freeze
  void setTimeout(int time_ms);

  static bool isEquivalent(const FlyCapture2::Property& prop1, const FlyCapture2::Property& prop2);

  /// Return the current frame rate
  double getFrameRate();

  /// Return the success ratio of retrieving frames
  double getSuccessRatio();

  /// Inversion of channels 1 and 2 (U and V)
  void invertChannels(cv::Mat& image);

  /// Returns a number of ms from a flycapture timestamp
  /// Value is always in [0,128]
  double timestamp2MS(FlyCapture2::TimeStamp ts);

  /// Retrieve current mode with the current
  FlyCapture2::Mode getMode();

  /// Retrieve current image format
  FlyCapture2::GigEImageSettings getImageSettings();

  /// Retrieve available formats
  FlyCapture2::GigEImageSettingsInfo getImageSettingsInfo();

  void setImagingMode(FlyCapture2::Mode mode);

  /// Update binning properties if required
  /// WARNING: updating binning can change the current imaging mode
  void updateBinning(unsigned int h_binning, unsigned int v_binning);

  /// Set current pixel format without changing other properties
  /// Note: dump(getImageSettingsInfo(), std::cout) to see available formats
  void setPixelFormat(FlyCapture2::PixelFormat pixel_format);

private:
  /**
   * PtGrey camera
   */
  FlyCapture2::GigECamera camera;

  /**
   * The specific id of the camera
   * WARNING: Using this information is required to do stereo vision
   * camera_id is defined by 4 unsigned ints
   */
  FlyCapture2::PGRGuid camera_id;

  /// Is the image stream starrted
  bool is_capturing;

  /// Number of success on RetrieveBuffer
  int nb_retrieve_success;

  /// Number of failures on RetrieveBuffer
  int nb_retrieve_failures;

  /// Last retrieval success in 'computer' time
  ::rhoban_utils::TimeStamp last_retrieval_success;

  /**
   * The values wished for properties
   * Property name -> Camera Property
   */
  std::map<std::string, FlyCapture2::Property> wished_properties;

  /**
   * Store Embedded values
   * Property name -> Camera Property
   */
  std::map<std::string, FlyCapture2::Property> properties;

  /**
   * Store informations on properties (such as: is auto supported, is manual
   * supported etc...)
   * Property name -> Camera Property information
   */
  std::map<std::string, FlyCapture2::PropertyInfo> properties_information;

  /**
   * From human names to enum values
   */
  static std::map<std::string, FlyCapture2::PropertyType> names_to_types;

  /**
   * Measure of the time spent since the last synch occured
   */
  unsigned int elapsed_from_synch_ms;

  /**
   * Delta between the pc timestamp and the camera timestamp
   */
  double ts_delta;

  /**
   * Usefull to do stuff the first time only
   */
  bool first_run;

  /**
   * Previous normalized timestamp, used to verify timing errors.
   */
  ::rhoban_utils::TimeStamp last_ts;

  /**
   * Sometimes a custom offset needs to be added to normalized_ts to fit the
   * modulo 128 s error which might happen
   */
  double custom_offset_ms;
};
}  // namespace Filters
}  // namespace Vision
