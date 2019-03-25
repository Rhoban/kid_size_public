#include "Filters/Source/SourceOpenCV.hpp"

#include "Filters/Pipeline.hpp"

#include <rhoban_utils/util.h>

namespace Vision
{
namespace Filters
{
SourceOpenCV::SourceOpenCV() : Source("SourceOpenCV"), device_index(-1)
{
}

SourceOpenCV::~SourceOpenCV()
{
}

Source::Type SourceOpenCV::getType() const
{
  return Type::Online;
}

std::string SourceOpenCV::getClassName() const
{
  return "SourceOpenCV";
}

int SourceOpenCV::expectedDependencies() const
{
  return 0;
}

void SourceOpenCV::process()
{
  if (!capture_device.isOpened())
  {
    throw std::runtime_error(DEBUG_INFO + " capture_device is not opened");
  }
  cv::Mat tmp_img;
  capture_device >> tmp_img;
  img() = tmp_img.clone();
  getPipeline()->setTimestamp(rhoban_utils::TimeStamp::now());
}

void SourceOpenCV::fromJson(const Json::Value& v, const std::string& dir_name)
{
  Source::fromJson(v, dir_name);
  rhoban_utils::tryRead(v, "device_name", &device_name);
  rhoban_utils::tryRead(v, "device_index", &device_index);

  bool index_provided = device_index >= 0;
  bool name_provided = device_name != "";

  if (capture_device.isOpened())
  {
    throw std::runtime_error(DEBUG_INFO + " already opened");
  }

  // Checking invalid json format
  if (index_provided && name_provided)
  {
    throw rhoban_utils::JsonParsingError(DEBUG_INFO + " both device_name and device_index provided," +
                                         " they are mutually exclusive");
  }
  else if ((!index_provided) && (!name_provided))
  {
    throw rhoban_utils::JsonParsingError(DEBUG_INFO + " none of device_name or device_index is provided.");
  }
  else if (name_provided)
  {
    if (!capture_device.open(device_name))
    {
      throw std::runtime_error(DEBUG_INFO + " failed to open device '" + device_name + "'");
    }
  }
  else
  {
    if (!capture_device.open(device_index + cv::CAP_ANY))
    {
      throw std::runtime_error(DEBUG_INFO + " failed to open device " + std::to_string(device_index) + "");
    }
  }
}

Json::Value SourceOpenCV::toJson() const
{
  Json::Value v = Source::toJson();
  v["device_name"] = device_name;
  return v;
}

}  // namespace Filters
}  // namespace Vision
