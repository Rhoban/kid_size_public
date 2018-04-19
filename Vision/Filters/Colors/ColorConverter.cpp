#include "Filters/Colors/ColorConverter.hpp"

#include <opencv2/imgproc/imgproc.hpp>

namespace Vision {
namespace Filters {

std::map<std::string, int> ColorConverter::strToCVCode;

void ColorConverter::process() {
  cv::cvtColor(*(getDependency().getImg()), img(), getConvCode());
}

void ColorConverter::fromJson(const Json::Value & v, const std::string & dir_name) {
  Filter::fromJson(v, dir_name);
  rhoban_utils::tryRead(v,"conversion",&conversion);
}

Json::Value ColorConverter::toJson() const {
  Json::Value v = Filter::toJson();
  v["conversion"] = conversion;
  return v;
}

int ColorConverter::getConvCode() {
  // Lazy builder
  if (strToCVCode.size() == 0) {
    buildMap();
  }
  try {
    return strToCVCode.at(conversion);
  } catch (const std::out_of_range &exc) {
    throw std::runtime_error("Conversion Code unknown for : " + conversion);
  }
}

void ColorConverter::buildMap() {
  strToCVCode.clear();
  strToCVCode["BGR2HSV"] = CV_BGR2HSV;
  strToCVCode["HSV2BGR"] = CV_HSV2BGR;
  strToCVCode["YCrCb2BGR"] = CV_YCrCb2BGR;
  strToCVCode["YCrCb2RGB"] = CV_YCrCb2RGB;
  strToCVCode["RGB2YCrCb"] = CV_RGB2YCrCb;
  strToCVCode["BGR2YCrCb"] = CV_BGR2YCrCb;
}
}
}
