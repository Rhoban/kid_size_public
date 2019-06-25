#include "SourcePtGrey.hpp"
#include "Utils/PtGreyExceptions.hpp"

#include "Filters/Pipeline.hpp"

#include "RhIO.hpp"

#include "rhoban_utils/timing/benchmark.h"
#include <rhoban_utils/logging/logger.h>
#include <rhoban_utils/util.h>

#include <opencv2/highgui/highgui.hpp>

#include <thread>
#include <unistd.h>
#include <iomanip>

using ::rhoban_utils::Benchmark;
using ::rhoban_utils::TimeStamp;
using namespace std;

static rhoban_utils::Logger logger("SourcePtGrey");

namespace Vision
{
namespace Filters
{
std::map<std::string, FlyCapture2::PropertyType> SourcePtGrey::names_to_types = {
  { "Brightness", FlyCapture2::BRIGHTNESS },
  { "Exposure", FlyCapture2::AUTO_EXPOSURE },
  { "Shutter", FlyCapture2::SHUTTER },
  { "FrameRate", FlyCapture2::FRAME_RATE },
  { "Gain", FlyCapture2::GAIN },
  { "WhiteBalance", FlyCapture2::WHITE_BALANCE }
};

std::map<unsigned int, std::string> pixel_format_names = {
  { FlyCapture2::PIXEL_FORMAT_MONO8, "Mono 8" },         { FlyCapture2::PIXEL_FORMAT_MONO12, "Mono 12" },
  { FlyCapture2::PIXEL_FORMAT_MONO16, "Mono 16" },       { FlyCapture2::PIXEL_FORMAT_RAW8, "Raw 8" },
  { FlyCapture2::PIXEL_FORMAT_RAW12, "Raw 12" },         { FlyCapture2::PIXEL_FORMAT_RAW16, "Raw 16" },
  { FlyCapture2::PIXEL_FORMAT_411YUV8, "YUV 411" },      { FlyCapture2::PIXEL_FORMAT_422YUV8, "YUV 422" },
  { FlyCapture2::PIXEL_FORMAT_444YUV8, "YUV 444" },      { FlyCapture2::PIXEL_FORMAT_RGB8, "RGB 8" },
  { FlyCapture2::PIXEL_FORMAT_RGB16, "RGB 16" },         { FlyCapture2::PIXEL_FORMAT_S_MONO16, "Signed Mono 16" },
  { FlyCapture2::PIXEL_FORMAT_S_RGB16, "Signed RGB 16" }
  // Ignoring vendor pixel format 422YUV8_JPEG
};

SourcePtGrey::SourcePtGrey()
  : Source("SourcePtGrey")
  , is_capturing(false)
  , nb_retrieve_success(0)
  , nb_retrieve_failures(0)
  , elapsed_from_synch_ms(0)
  , ts_delta(0.0)
  , first_run(true)
  , last_ts()
  , custom_offset_ms(0.0)
{
  last_retrieval_success = TimeStamp::now();
}

SourcePtGrey::~SourcePtGrey()
{
}

FlyCapture2::Property SourcePtGrey::propertyFromJson(const Json::Value& v, FlyCapture2::PropertyType type) const
{
  bool isWhiteBalance = (type == FlyCapture2::PropertyType::WHITE_BALANCE);
  FlyCapture2::Property p;
  p.type = type;
  // default values
  p.present = true;
  p.absControl = !isWhiteBalance;
  p.onePush = false;
  p.onOff = true;
  p.autoManualMode = false;  // false -> manual
  p.valueA = 0;
  p.valueB = 0;
  p.autoManualMode = rhoban_utils::read<bool>(v, "autoManualMode");
  // Reading using temporary variables due to type
  if (!isWhiteBalance)
  {
    double tmpAbsVal = 0;
    rhoban_utils::tryRead(v, "absValue", &tmpAbsVal);
    p.absValue = tmpAbsVal;
  }
  else
  {
    int tmpValA = 0;
    int tmpValB = 0;
    rhoban_utils::tryRead(v, "valueA", &tmpValA);
    rhoban_utils::tryRead(v, "valueB", &tmpValB);
    p.valueA = tmpValA;
    p.valueB = tmpValB;
  }
  return p;
}

double SourcePtGrey::getFrameRate()
{
  return properties.at("FrameRate").absValue;
}

double SourcePtGrey::getSuccessRatio()
{
  int nb_retrievals = nb_retrieve_success + nb_retrieve_failures;
  if (nb_retrievals == 0)
    return 1;
  return nb_retrieve_success / (double)nb_retrievals;
}

void SourcePtGrey::fromJson(const Json::Value& v, const std::string& dir_name)
{
  Filter::fromJson(v, dir_name);
  Json::Value wp_val = v["wished_properties"];
  // If wished properties are found, use them
  if (!wp_val.isNull())
  {
    if (!wp_val.isObject())
    {
      throw rhoban_utils::JsonParsingError("SourcePtGrey::fromJson: 'wished_properties' is not an object");
    }
    for (const auto& entry : names_to_types)
    {
      const std::string& prop_name = entry.first;
      FlyCapture2::PropertyType prop_type = entry.second;
      Json::Value prop_val = wp_val[prop_name];
      if (!prop_val.isNull())
      {
        wished_properties[prop_name] = propertyFromJson(prop_val, prop_type);
      }
    }
  }
}

Json::Value SourcePtGrey::toJson() const
{
  return Filter::toJson();
  // TODO
}

string SourcePtGrey::getClassName() const
{
  return "SourcePtGrey";
}

int SourcePtGrey::expectedDependencies() const
{
  return 0;
}

Source::Type SourcePtGrey::getType() const
{
  return Type::Online;
}

void SourcePtGrey::updateImageSettings()
{
  // Error variable
  setImagingMode(FlyCapture2::Mode::MODE_1);
  updateBinning(2, 2);
  setPixelFormat(FlyCapture2::PixelFormat::PIXEL_FORMAT_444YUV8);
}

void SourcePtGrey::updatePacketProperties()
{
  // Prepare packets
  // Using larger packets reduces the number of CPU interruptions
  FlyCapture2::GigEProperty packet_size_prop;
  packet_size_prop.propType = FlyCapture2::PACKET_SIZE;
  packet_size_prop.value = 9000;
  FlyCapture2::GigEProperty packet_delay_prop;
  packet_delay_prop.propType = FlyCapture2::PACKET_DELAY;
  // In 640*480, at 25 fps, with 3 bytes per pixel, the required bandwidth is:
  // 640 * 480 * 25 * 3 ~= 24 * 10^6 <- 23 MB/s
  // Some examples of combinations packet_delay, packet_size and bandwidth are
  // provided in the Technical reference guide of blackfly:
  // section:  "Determining Bandwidth requirements"
  // 55 MB: (9000,1800) or (1400,255)
  // 25 MB: (9000,5900) or (1400,900)
  //
  // Since no obvious relationships between Baudrate and value is exhibited,
  // setting this value is subjet to extreme caution
  // - If the value is too low: Risk of 'dropping frames'
  // - If the value is too high: Unknown consequences
  packet_delay_prop.value = 6000;

  // Send packets
  FlyCapture2::Error error;
  error = camera.SetGigEProperty(&packet_size_prop);
  if (error != FlyCapture2::PGRERROR_OK)
  {
    std::ostringstream oss;
    oss << "SourcePtGrey::updatePacketProperties: failed to set packet size: " << error.GetType();
    {
      oss << ": Camera was capturing, stop capturing before changing packet "
             "size";
    }
    throw Utils::PtGreyException(oss.str());
  }
  error = camera.SetGigEProperty(&packet_delay_prop);
  if (error != FlyCapture2::PGRERROR_OK)
  {
    std::ostringstream oss;
    oss << "SourcePtGrey::updatePacketProperties: failed to set packet delay";
    if (is_capturing)
    {
      oss << ": Camera was capturing, stop capturing before changing packet "
             "size";
    }
    throw Utils::PtGreyException(oss.str());
  }
}

void SourcePtGrey::startCamera()
{
  FlyCapture2::Error error;
  // If connected, disconnect
  if (camera.IsConnected())
  {
    camera.Disconnect();
  }
  // Connect to camera
  error = camera.Connect(0);  // TODO: replace by &camera_id when implemented
  if (error != FlyCapture2::PGRERROR_OK)
  {
    const std::string message =
        "SourcePtGrey::StartCamera: Failed to connect to camera. Message: " + std::string(error.GetDescription());
    throw Utils::PtGreyConnectionException(message);
  }
  try
  {
    // Properly set up size of packet and delay between packets
    updatePacketProperties();
    // Set appropriate mode and size
    updateImageSettings();
    // Ensure that if connection is lost we will not be stopped
    setTimeout(500);
    // Start capture
    while (true)
    {
      error = camera.StartCapture();
      if (error == FlyCapture2::PGRERROR_ISOCH_BANDWIDTH_EXCEEDED)
      {
        throw Utils::PtGreyException("SourcePtGrey::StartCamera: bandwidth exceeded");
      }
      else if (error == FlyCapture2::PGRERROR_ISOCH_ALREADY_STARTED)
      {
        std::cerr << "SourcePtGrey::StartCamera: Isoch already started: stopping" << std::endl;
        camera.StopCapture();
      }
      else if (error != FlyCapture2::PGRERROR_OK)
      {
        std::cerr << "SourcePtGrey::StartCamera: Failed to start image capture: '" << error.GetDescription();
        reconnectCamera();
      }
      else
      {
        break;
      }
    }
    logger.log("Capturing OK");
    is_capturing = true;
    // Update internal properties
    updateProperties();
    logger.log("Update properties OK");
    updatePropertiesInformation();
    logger.log("Update properties information OK");
    // Apply wished properties
    applyWishedProperties();
    logger.log("ApplyWishedProperties OK");
    // Once properties have been read and applied, initialize them
    // TODO: instead, wished_properties should be filled with entries which have
    // not been presented yet
    wished_properties = properties;
    // DEBUG print for properties
    // dumpProperties(std::cout);
    // dumpPropertiesInformation(std::cout);
    bindProperties();
    logger.log("bindProperties OK");

    // TODO: extract code somewhere else
    FlyCapture2::EmbeddedImageInfo embeddedInfo;
    embeddedInfo.timestamp.onOff = true;
    error = camera.SetEmbeddedImageInfo(&embeddedInfo);
    if (error != FlyCapture2::PGRERROR_OK)
    {
      throw Utils::PtGreyException("failed to set 'embedded ImageInfo'");
    }
    logger.log("setEmbeddedImageInfo OK");
  }
  catch (const Utils::PtGreyException& exc)
  {
    logger.error("Got a PtGreyException while preparing camera: '%s'", exc.what());
    camera.Disconnect();
    throw;
  }
}

void SourcePtGrey::endCamera()
{
  FlyCapture2::Error error;
  is_capturing = false;
  if (is_capturing)
  {
    camera.StopCapture();
    if (error != FlyCapture2::PGRERROR_OK)
    {
      throw Utils::PtGreyException("SourcePtGrey::endCamera: Failed to stop capture!");
    }
  }
  if (camera.IsConnected())
  {
    camera.Disconnect();
  }
}

void SourcePtGrey::reconnectCamera()
{
  FlyCapture2::Error error;
  logger.log("reconnectCamera:IsConnected()");
  // If connected, disconnect
  if (camera.IsConnected())
  {
    logger.log("reconnectCamera::Disconnect()");
    camera.Disconnect();
  }
  // Connect to camera
  logger.log("reconnectCamera::Connect()");
  error = camera.Connect(0);  // TODO: replace by &camera_id when implemented
  if (error != FlyCapture2::PGRERROR_OK)
  {
    throw Utils::PtGreyConnectionException("SourcePtGrey::StartCamera: Failed to connect to camera");
  }
  logger.log("reconnectCamera::OK()");
}

void SourcePtGrey::updateProperties()
{
  for (const auto& entry : names_to_types)
  {
    std::string property_name = entry.first;
    FlyCapture2::PropertyType property_type = entry.second;
    // Read informations from camera
    FlyCapture2::Error error;
    FlyCapture2::Property property;
    property.type = property_type;
    error = camera.GetProperty(&property);
    if (error != FlyCapture2::PGRERROR_OK)
    {
      throw Utils::PtGreyException("Failed to Get Property: '" + property_name + "'");
    }
    // Write Property inside local map
    properties[property_name] = property;
  }
}

void SourcePtGrey::updatePropertiesInformation()
{
  for (const auto& entry : names_to_types)
  {
    std::string property_name = entry.first;
    FlyCapture2::PropertyType property_type = entry.second;
    // Read informations from camera
    FlyCapture2::Error error;
    FlyCapture2::PropertyInfo property_info;
    property_info.type = property_type;
    error = camera.GetPropertyInfo(&property_info);
    if (error != FlyCapture2::PGRERROR_OK)
    {
      throw Utils::PtGreyException("Failed to Get Property Information: '" + property_name + "'");
    }
    // Write Property inside local map
    properties_information[property_name] = property_info;
  }
}

void SourcePtGrey::dump(const FlyCapture2::Format7ImageSettings& s, std::ostream& out)
{
  out << "----------------" << std::endl;
  out << "F7 Image Settings" << std::endl;
  out << "----------------" << std::endl;
  out << "mode: " << s.mode << std::endl;
  out << "offset: (" << s.offsetX << "," << s.offsetY << ")" << std::endl;
  out << "size: (" << s.width << "x" << s.height << ")" << std::endl;
  out << "pixelFormat: " << pixel_format_names.at(s.pixelFormat) << std::endl;
}

void SourcePtGrey::dump(const FlyCapture2::GigEImageSettings& s, std::ostream& out)
{
  out << "--------------" << std::endl;
  out << "Image Settings" << std::endl;
  out << "--------------" << std::endl;
  out << "offset: (" << s.offsetX << "," << s.offsetY << ")" << std::endl;
  out << "size: (" << s.width << "x" << s.height << ")" << std::endl;
  out << "pixelFormat: " << pixel_format_names.at(s.pixelFormat) << std::endl;
}

void SourcePtGrey::dump(const FlyCapture2::GigEImageSettingsInfo& image_settings_info, std::ostream& out)
{
  out << "-------------------" << std::endl;
  out << "Image Settings Info" << std::endl;
  out << "-------------------" << std::endl;
  for (const auto& entry : pixel_format_names)
  {
    unsigned int format_mask = entry.first;
    const std::string& format_name = entry.second;
    if ((format_mask & image_settings_info.pixelFormatBitField) != 0)
    {
      out << "\t" << format_name << std::endl;
    }
  }
}

void SourcePtGrey::dumpProperties(std::ostream& out)
{
  out << "----------" << std::endl;
  out << "Properties" << std::endl;
  out << "----------" << std::endl;
  for (const auto& entry : properties)
  {
    out << entry.first << std::endl;
    writeProperty(entry.second, out, "  ");
  }
}

void SourcePtGrey::dumpPropertiesInformation(std::ostream& out)
{
  out << "---------------------" << std::endl;
  out << "PropertiesInformation" << std::endl;
  out << "---------------------" << std::endl;
  for (const auto& entry : properties_information)
  {
    out << entry.first << std::endl;
    writePropertyInformation(entry.second, out, "  ");
  }
}

FlyCapture2::PropertyType SourcePtGrey::getPropertyType(const std::string& name)
{
  try
  {
    return names_to_types.at(name);
  }
  catch (const std::out_of_range& exc)
  {
    throw std::out_of_range("SourcePtGrey::getPropertyType: unknown name '" + name + "'");
  }
}

void SourcePtGrey::writeProperty(const FlyCapture2::Property& property, std::ostream& out, const std::string& prefix)
{
  out << prefix << "type            : " << property.type << std::endl;
  out << prefix << "present         : " << property.present << std::endl;
  out << prefix << "absolute control: " << property.absControl << std::endl;
  out << prefix << "one push        : " << property.onePush << std::endl;
  out << prefix << "on/off          : " << property.onOff << std::endl;
  out << prefix << "auto            : " << property.autoManualMode << std::endl;
  out << prefix << "value A         : " << property.valueA << std::endl;
  out << prefix << "value B         : " << property.valueB << std::endl;
  out << prefix << "absolute value  : " << property.absValue << std::endl;
}

void SourcePtGrey::writePropertyInformation(const FlyCapture2::PropertyInfo& property_info, std::ostream& out,
                                            const std::string& prefix)
{
  out << prefix << "type            : " << property_info.type << std::endl;
  out << prefix << "present         : " << property_info.present << std::endl;
  out << prefix << "support auto    : " << property_info.autoSupported << std::endl;
  out << prefix << "support manual  : " << property_info.manualSupported << std::endl;
  out << prefix << "support on/off  : " << property_info.onOffSupported << std::endl;
  out << prefix << "support one push: " << property_info.onePushSupported << std::endl;
  out << prefix << "support abs val : " << property_info.absValSupported << std::endl;
  out << prefix << "support read out: " << property_info.readOutSupported << std::endl;
  out << prefix << "min             : " << property_info.min << std::endl;
  out << prefix << "max             : " << property_info.max << std::endl;
  out << prefix << "abs min         : " << property_info.absMin << std::endl;
  out << prefix << "abs max         : " << property_info.absMax << std::endl;
  out << prefix << "Units (long)    : " << property_info.pUnits << std::endl;
  out << prefix << "Units (short)   : " << property_info.pUnitAbbr << std::endl;
}

void SourcePtGrey::bindProperties()
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
    node.newFloat("ts_delta")->defaultValue(ts_delta);
    node.newFloat("normalized_ts")->defaultValue(last_ts.getTimeMS());
    node.newFloat("custom_offset")->defaultValue(custom_offset_ms);
  }
  // Declare all field in specific folder
  for (const auto& entry : wished_properties)
  {
    const std::string& property_name = entry.first;
    std::string property_path = filter_path + "/" + property_name;
    FlyCapture2::Property property = entry.second;
    FlyCapture2::PropertyInfo property_info = properties_information.at(property_name);
    // If property has already been created, skip property
    if (RhIO::Root.childExist(property_path))
      continue;
    // Create Property node
    RhIO::Root.newChild(property_path);
    RhIO::IONode& node = RhIO::Root.child(property_path);
    // Create a node for each property
    node.newBool("auto")->defaultValue(false);
    if (property.type == FlyCapture2::WHITE_BALANCE)
    {
      node.newInt("valueA")->defaultValue(property.valueA)->minimum(0)->maximum(1023);
      node.newInt("valueB")->defaultValue(property.valueB)->minimum(0)->maximum(1023);
    }
    else
    {
      node.newBool("absControl")->defaultValue(true);
      node.newFloat("absValue")
          ->defaultValue(property.absValue)
          ->minimum(property_info.absMin)
          ->maximum(property_info.absMax);
    }
    // TODO handle other types
  }
}

void SourcePtGrey::importPropertiesFromRhIO()
{
  std::string filter_path = rhio_path + getName();

  // Declare all field in specific folder
  for (auto& entry : wished_properties)
  {
    const std::string& property_name = entry.first;
    FlyCapture2::Property& property = entry.second;
    std::string property_path = filter_path + "/" + property_name;
    RhIO::IONode& node = RhIO::Root.child(property_path);
    property.autoManualMode = node.getValueBool("auto").value;
    if (property.type == FlyCapture2::WHITE_BALANCE)
    {
      property.absControl = false;
      property.valueA = node.getValueInt("valueA").value;
      property.valueB = node.getValueInt("valueB").value;
    }
    else
    {
      property.absControl = node.getValueBool("absControl").value;
      property.absValue = node.getValueFloat("absValue").value;
    }
  }
}

void SourcePtGrey::updateRhIO()
{
  std::string filter_path = rhio_path + getName();

  RhIO::IONode& monitoring_node = RhIO::Root.child(filter_path + "/monitoring");
  monitoring_node.setInt("success", nb_retrieve_success);
  monitoring_node.setInt("failures", nb_retrieve_failures);
  monitoring_node.setFloat("ratio", getSuccessRatio());
  monitoring_node.setFloat("ts_delta", ts_delta);
  monitoring_node.setFloat("normalized_ts", last_ts.getTimeMS());
  monitoring_node.setFloat("custom_offset", custom_offset_ms);
}

void SourcePtGrey::applyWishedProperties()
{
  std::string filter_path = rhio_path + getName();

  // Declare all field in specific folder
  for (auto& entry : wished_properties)
  {
    const std::string& property_name = entry.first;
    FlyCapture2::Property& wished_property = entry.second;
    // Lazy application of parameters
    if (!isEquivalent(wished_property, properties.at(property_name)))
    {
      std::cout << "Difference found in configuration for property: '" << property_name << "'" << std::endl;
      if (wished_property.type == FlyCapture2::WHITE_BALANCE)
      {
        wished_property.absControl = false;
      }
      FlyCapture2::Error error;
      // Uploading property to camera
      error = camera.SetProperty(&wished_property, false);
      if (error != FlyCapture2::PGRERROR_OK)
      {
        dumpProperties(std::cout);
        // dumpPropertiesInformation(std::cout);
        std::cout << "PtGreyError: " << error.GetDescription() << std::endl;
        std::cout << "Wished prop: ";
        writeProperty(entry.second, std::cout, "  ");
        throw Utils::PtGreyException("Failed to apply property: '" + property_name + "'");
      }
      // Checking that property has properly been updated
      error = camera.GetProperty(&(properties.at(property_name)));
      if (error != FlyCapture2::PGRERROR_OK)
      {
        throw Utils::PtGreyException("Failed to get property: '" + property_name + "'");
      }
      std::cout << "Difference between values:" << (wished_property.absValue - properties.at(property_name).absValue)
                << std::endl;
    }
  }
}

void SourcePtGrey::setTimeout(int time_ms)
{
  FlyCapture2::Error error;
  FlyCapture2::FC2Config config;
  // Grab config
  error = camera.GetConfiguration(&config);
  if (error != FlyCapture2::PGRERROR_OK)
  {
    std::ostringstream oss;
    oss << "SourcePtGrey::setTimeout: failed to getConfig: " << error.GetDescription();
    throw Utils::PtGreyException(oss.str());
  }
  config.grabTimeout = time_ms;
  // Set timeout
  error = camera.SetConfiguration(&config);
  if (error != FlyCapture2::PGRERROR_OK)
  {
    std::ostringstream oss;
    oss << "SourcePtGrey::setTimeout: failed to setConfig: " << error.GetDescription();
    throw Utils::PtGreyException(oss.str());
  }
}

// The camera clock crystal, a CITIZEN CS10‐25.000MABJTR has a drift specification of ± 50 PPM
// Which means that after 20 minutes, we can expect the clock to drift +-60ms.
//=> If we want a ~1ms precision, we should call this function at least every 24 seconds.
double SourcePtGrey::measureTimestampDelta()
{
  // This function costs ~3ms on the fitlet.

  /**How to estimate the delay between the capture moment and the image
   * reception moment? Sure, we have a precise, image embedded timsestamp, but
   * the clock of the camera and the clock of the PC are not synchronized. A
   * decent enough solution is to request the timestamp directly to the camera
   * (and not the value embedded in an image). Approximation : if t0 is the
   * local timestamp when we sent the order, and t1 is the local timestap when
   * we received the camera timestamp, then the received timestamp matches
   * (t0+t1)/2.
   */

  TimeStamp t0 = TimeStamp::now();
  // We need to ask the camera to latch (=flush) the timestamp
  unsigned long latch_data = 0x1 << 1;

  /* tl dr do not reset_data.
   * Weirdly enough, the timestamp embedded into the frames has 32 bits (resets every 128 secs) and
   * the latched camera timestamp has 64 bits. Their LSB seems to be the same. Their value seems also
   * to be the same (modulo 128sec) UNLESS a reset was asked on the latched timsestamp which causes
   * both timers to desynchronize.
   */
  /*
  unsigned long reset_data = 0x1;
  error = camera.WriteGVCPRegister(0x0944, reset_data);
  if (error != FlyCapture2::PGRERROR_OK) {
    const std::string message = "WriteGVCPRegister failed (timestamp reset). Message: " +
  std::string(error.GetDescription()); throw PtGreyException(message);
    }*/

  FlyCapture2::Error error = camera.WriteGVCPRegister(0x0944, latch_data);
  if (error != FlyCapture2::PGRERROR_OK)
  {
    const std::string message =
        "WriteGVCPRegister failed (timestamp latch). Message: " + std::string(error.GetDescription());
    if (diffMs(last_retrieval_success, TimeStamp::now()) > 5000)
    {
      endCamera();
      throw Utils::PtGreyException("Failed to reset timestamp for more than 5 s");
    }
    throw Utils::PtGreyException(message);
  }

  TimeStamp t1 = TimeStamp::now();
  double elapsed = diffMs(t0, t1);

  // We can read the latched value now
  unsigned int addressHigh = 0x0948;
  unsigned int addressLow = 0x094C;
  unsigned int bufferLow = 0;
  unsigned int bufferHigh = 0;
  error = camera.ReadGVCPRegister(addressHigh, &bufferHigh);
  if (error != FlyCapture2::PGRERROR_OK)
  {
    const std::string message = "ReadRegister failed (timestamp). Message: " + std::string(error.GetDescription());
    throw Utils::PtGreyException(message);
  }
  error = camera.ReadGVCPRegister(addressLow, &bufferLow);
  if (error != FlyCapture2::PGRERROR_OK)
  {
    const std::string message = "ReadRegister failed (timestamp). Message: " + std::string(error.GetDescription());
    throw Utils::PtGreyException(message);
  }

  // From latched timestamp to milliseconds. The timestamp increments at 125MHz
  double low = bufferLow / (double)125000;
  double high = bufferHigh * (((unsigned long long int)1 << 32) / (double)125000);
  double latchedMs = high + low;
  latchedMs = fmod(latchedMs, 128000.0);
  double localTimestamp = t0.getTimeMS() + elapsed / 2;
  double delta = localTimestamp - latchedMs;

  return delta;
}

void SourcePtGrey::process()
{
  FlyCapture2::Image fc_image;
  // If capture is not activated yet, start camera
  if (!is_capturing)
  {
    startCamera();
  }

  // Import parameters from> rhio and apply them if necessary
  importPropertiesFromRhIO();
  applyWishedProperties();

  // Show elapsed time since last call
  TimeStamp now = TimeStamp::now();
  // TODO require to store lastRetrievalAttempt
  //     - (accumulation on elapsed when failing retrieval)
  double elapsed = diffMs(last_retrieval_success, now);
  elapsed_from_synch_ms = elapsed_from_synch_ms + elapsed;

  // TODO set as a json parameter?
  if ((elapsed_from_synch_ms > 10000) | first_run)
  {
    if (first_run)
    {
      first_run = false;
      last_ts = TimeStamp::fromMS(0);
    }

    elapsed_from_synch_ms = 0;
    // Re-synch the pc timestamp with the camera timestamp
    ts_delta = measureTimestampDelta();
  }

  // Grab frame from camera
  FlyCapture2::Error error = camera.RetrieveBuffer(&fc_image);

  if (error == FlyCapture2::PGRERROR_TIMEOUT)
  {
    endCamera();
    throw Utils::PtGreyException("RetrieveBuffer timed out");
  }
  else if (error != FlyCapture2::PGRERROR_OK)
  {
    std::ostringstream oss;
    oss << "Failed buffer retrieval: '" << error.GetDescription() << "'";
    // Detailed error
    // oss << "Failed to retrieve buffer from PtGrey: | "
    //    << "Error description: " << error.GetDescription() << " | "
    //    << "Elapsed time since last retrieval: " << elapsed << " ms |"
    //    << "Elapsed from synch " << elapsed_from_synch_ms;
    nb_retrieve_failures++;
    updateRhIO();
    throw Utils::PtGreyException(oss.str());
  }
  last_retrieval_success = now;
  nb_retrieve_success++;

  Benchmark::open("Copy image");
  unsigned int bytes_per_row = fc_image.GetReceivedDataSize() / fc_image.GetRows();
  cv::Mat tmp_img = cv::Mat(fc_image.GetRows(), fc_image.GetCols(), CV_8UC3, fc_image.GetData(), bytes_per_row).clone();
  // TODO : skip the invertChannels step to reduce CPU usage.
  invertChannels(tmp_img);
  img() = tmp_img;
  Benchmark::close("Copy image");

  // TimeStamp
  FlyCapture2::TimeStamp ts = fc_image.GetTimeStamp();

  double image_ts_ms = timestamp2MS(ts);
  double normalized_frame_ts = image_ts_ms + ts_delta;

  double now_ms = TimeStamp::now().getTimeMS();

  custom_offset_ms = 0;
  if ((now_ms - normalized_frame_ts) > 128000)
  {
    // ts_delta was measured before the 128000ms counter reseted.
    // Meanwhile, the ts of the current image reseted.
    custom_offset_ms = 128000;
  }
  normalized_frame_ts += custom_offset_ms;

  // Never allow to publish images more recent than current time!
  double frame_age_ms = now_ms - normalized_frame_ts;
  if (frame_age_ms < 0)
  {
    std::ostringstream oss;
    oss << "SourcePtGrey::process: frame is dated from " << (-frame_age_ms) << " ms in the future -> refused";
    measureTimestampDelta();
    throw Utils::PtGreyException(oss.str());
  }
  if (frame_age_ms > 128000)
  {
    std::ostringstream oss;
    oss << "SourcePtGrey::process: frame is dated from " << frame_age_ms << " ms in the past -> too old";
    measureTimestampDelta();
    throw Utils::PtGreyException(oss.str());
  }

  TimeStamp frame_ts = TimeStamp::fromMS(normalized_frame_ts);
  getPipeline()->setTimestamp(frame_ts);

  double elapsed_ms = diffMs(last_ts, frame_ts);
  if (elapsed_ms <= 0)
  {
    updateRhIO();
    std::ostringstream oss;
    oss << "Invalid elapsed time: " << elapsed_ms << " (Elapsed from sync " << elapsed_from_synch_ms << ")";
    throw Utils::PtGreyException(oss.str());
  }
  else if (elapsed_ms > 500)
  {
    std::cout << "SourcePtGrey:: Warning: Elapsed time: " << elapsed_ms << " ms" << std::endl;
  }
  last_ts = TimeStamp::fromMS(normalized_frame_ts);

  updateRhIO();
}

void SourcePtGrey::invertChannels(cv::Mat& frame)
{
  uint8_t* data = (uint8_t*)frame.data;

  for (int i = 0; i < frame.cols * frame.rows; i++)
  {
    // According to: https://www.ptgrey.com/KB/10092
    // Format of PtGrey is UYV, but definition of U and V might be swapped
    uint8_t V = data[3 * i + 0];
    uint8_t Y = data[3 * i + 1];
    uint8_t U = data[3 * i + 2];
    data[3 * i + 0] = Y;
    data[3 * i + 1] = U;
    data[3 * i + 2] = V;
  }
}

bool SourcePtGrey::isEquivalent(const FlyCapture2::Property& prop1, const FlyCapture2::Property& prop2)
{
  if (prop1.type == FlyCapture2::WHITE_BALANCE)
  {
    return prop1.valueA == prop2.valueA && prop1.valueB == prop2.valueB && prop1.autoManualMode == prop2.autoManualMode;
  }
  double abs_value_tol = 0.02;  // TODO! Use something specific to each property
  return (prop1.absControl == prop2.absControl && prop1.autoManualMode == prop2.autoManualMode &&
          std::fabs(prop1.absValue - prop2.absValue) <= abs_value_tol);
}

double SourcePtGrey::timestamp2MS(FlyCapture2::TimeStamp ts)
{
  // Second_count increments from 0 to 127, counting the number of seconds.
  // Encoded on 7 bits.
  // Cycle_count increments from 0 to 7999, which equals one second. Encoded on
  // 13 bits, but resets after 7999.
  // Cycle offset is encoded on 12 bits. It's the most accurate
  // portion of the timestamp, reseting every 125us. However, the Timestamp Tick
  // Frequency is by default 125MHz, which would give a reset of the 12 last bits every 62.5us.
  unsigned int second_count = ts.cycleSeconds;
  unsigned int cycle_count = ts.cycleCount;
  unsigned int cycle_offset = ts.cycleOffset;
  double ms =
      1000 * ((double)second_count + (double)cycle_count / 8000.0 + (double)cycle_offset * 0.000000030517578125);

  return ms;
}

FlyCapture2::Mode SourcePtGrey::getMode()
{
  FlyCapture2::Mode image_mode;
  FlyCapture2::Error error = camera.GetGigEImagingMode(&image_mode);
  if (error != FlyCapture2::ErrorType::PGRERROR_OK)
  {
    throw Utils::PtGreyException(DEBUG_INFO + error.GetDescription());
  }
  return image_mode;
}

FlyCapture2::GigEImageSettings SourcePtGrey::getImageSettings()
{
  FlyCapture2::GigEImageSettings settings;
  FlyCapture2::Error error = camera.GetGigEImageSettings(&settings);
  if (error != FlyCapture2::ErrorType::PGRERROR_OK)
  {
    throw Utils::PtGreyException(DEBUG_INFO + error.GetDescription());
  }
  return settings;
}

FlyCapture2::GigEImageSettingsInfo SourcePtGrey::getImageSettingsInfo()
{
  FlyCapture2::GigEImageSettingsInfo infos;
  FlyCapture2::Error error = camera.GetGigEImageSettingsInfo(&infos);
  if (error != FlyCapture2::ErrorType::PGRERROR_OK)
  {
    throw Utils::PtGreyException(DEBUG_INFO + error.GetDescription());
  }
  return infos;
}

void SourcePtGrey::setImagingMode(FlyCapture2::Mode mode)
{
  FlyCapture2::Error error = camera.SetGigEImagingMode(mode);
  if (error != FlyCapture2::ErrorType::PGRERROR_OK)
  {
    throw Utils::PtGreyException(DEBUG_INFO + error.GetDescription());
  }
}

void SourcePtGrey::updateBinning(unsigned int h_binning, unsigned int v_binning)
{
  // Retrieving current binning properties
  unsigned int current_h_binning = 0;
  unsigned int current_v_binning = 0;
  FlyCapture2::Error error;
  error = camera.GetGigEImageBinningSettings(&current_h_binning, &current_v_binning);
  if (error != FlyCapture2::ErrorType::PGRERROR_OK)
  {
    throw Utils::PtGreyException(DEBUG_INFO + "Error getting current binning settings" + error.GetDescription());
  }

  // Setting binning
  if (h_binning != current_h_binning || v_binning != current_v_binning)
  {
    error = camera.SetGigEImageBinningSettings(h_binning, v_binning);
    if (error != FlyCapture2::ErrorType::PGRERROR_OK)
    {
      throw Utils::PtGreyException(DEBUG_INFO + "Error setting binning settings" + error.GetDescription());
    }
  }
}

void SourcePtGrey::setPixelFormat(FlyCapture2::PixelFormat pixel_format)
{
  struct FlyCapture2::GigEImageSettings image_settings = getImageSettings();
  image_settings.pixelFormat = pixel_format;
  FlyCapture2::Error error = camera.SetGigEImageSettings(&image_settings);
  if (error != FlyCapture2::ErrorType::PGRERROR_OK)
  {
    throw Utils::PtGreyException(DEBUG_INFO + "Error setting image settings: " + error.GetDescription());
  }
}

}  // namespace Filters
}  // namespace Vision
