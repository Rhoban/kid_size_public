#include <stdexcept>
#include "Filters/Source/SourceLogs.hpp"

#include "rhoban_utils/timing/time_stamp.h"

#include <opencv2/opencv.hpp>

#include "Filters/Pipeline.hpp"

namespace Vision
{
namespace Filters
{
void SourceLogs::openImageSequence()
{
  images.loadImages(imagesFile);
  images.setIndex(startIndex);
}

void SourceLogs::process()
{
  images.nextImg();
  updateImg();
}

void SourceLogs::updateImg()
{
  img() = images.getImg().clone();
  if (display)
  {
    std::cout << "Displaying image : " << images.getIndex() << std::endl;
    std::cout << "-> Time: " << images.getTimestamp() << std::endl;
  }
  getPipeline()->setTimestamp(::rhoban_utils::TimeStamp::fromMS(images.getTimestamp()));
}

void SourceLogs::fromJson(const Json::Value& v, const std::string& dir_name)
{
  Filter::fromJson(v, dir_name);
  rhoban_utils::tryRead(v, "startIndex", &startIndex);
  rhoban_utils::tryRead(v, "imagesFile", &imagesFile);
  openImageSequence();
}

Json::Value SourceLogs::toJson() const
{
  Json::Value v = Filter::toJson();
  v["startIndex"] = startIndex;
  v["imagesFile"] = imagesFile;
  return v;
}

Source::Type SourceLogs::getType() const
{
  return Type::Log;
}

std::string SourceLogs::getImgName() const
{
  return images.imgOriginalName();
}

int SourceLogs::getIndex() const
{
  return images.getIndex();
}

bool SourceLogs::isValid() const
{
  return images.isValid();
}

void SourceLogs::previous()
{
  images.previousImg();
  updateImg();
}

void SourceLogs::update()
{
  updateImg();
}

int64_t SourceLogs::getClockOffset() const
{
  return images.getClockOffset();
}

}  // namespace Filters

}  // namespace Vision
