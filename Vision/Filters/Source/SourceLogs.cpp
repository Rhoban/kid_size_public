#include <stdexcept>
#include "Filters/Source/SourceLogs.hpp"

#include "rhoban_utils/timing/time_stamp.h"

#include <opencv2/opencv.hpp>

#include "Filters/Pipeline.hpp"

namespace Vision {
namespace Filters {

SourceLogs::SourceLogs(const std::string &name, const std::string &logFile,
                       int sIndex, Frequency::type frequency)
    : Source(name, Dependencies(), frequency), 
      startIndex(sIndex), imagesFile(logFile), images() {
  openImageSequence();
}

void SourceLogs::openImageSequence() {
  images.loadImages(imagesFile);
  images.setIndex(startIndex);
}

void SourceLogs::process() {
  try {
    images.nextImg();
    updateImg();
  } catch (const std::runtime_error &exc) {
    setErrorImg(exc.what());
  } catch (const std::out_of_range &exc) {
    setErrorImg(exc.what());
  }
  //_pipeline->updateGlobalCS(_sourceTimestamp);
}

void SourceLogs::updateImg() {
  img() = images.getImg().clone();
  if (debugLevel.graphics) {
    std::cout << "Displaying image : " << images.getIndex() << std::endl;
    std::cout << "-> Time: " << images.getTimestamp() << std::endl;

  }
  getPipeline()->setTimestamp(::rhoban_utils::TimeStamp::fromMS(images.getTimestamp()));
}

void SourceLogs::setErrorImg(const std::string &errorMsg) {
  img() = cv::Mat(600, 600, CV_8UC3, cv::Scalar(0, 0, 0));
  cv::putText(img(), errorMsg, cv::Point(img().cols / 3, img().rows / 2),
              cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255, 255, 255), 4);
  getPipeline()->setTimestamp(::rhoban_utils::TimeStamp::fromMS(0));
  throw std::runtime_error(errorMsg);
}

void SourceLogs::fromJson(const Json::Value & v, const std::string & dir_name) {
  Filter::fromJson(v, dir_name);
  rhoban_utils::tryRead(v,"startIndex",&startIndex);
  rhoban_utils::tryRead(v,"imagesFile",&imagesFile);
  openImageSequence();
}

Json::Value SourceLogs::toJson() const {
  Json::Value v = Filter::toJson();
  v["startIndex"] = startIndex;
  v["imagesFile"] = imagesFile;
  return v;
}

Source::Type SourceLogs::getType() const {
  return Type::Log;
}


std::string SourceLogs::getImgName() const { return images.imgOriginalName(); }

int SourceLogs::getIndex() const { return images.getIndex(); }

bool SourceLogs::isValid() const { return images.isValid(); }

void SourceLogs::previous() {
  try {
    images.previousImg();
    updateImg();
  } catch (const std::runtime_error &exc) {
    setErrorImg(exc.what());
  } catch (const std::out_of_range &exc) {
    setErrorImg(exc.what());
  }
}

void SourceLogs::update() { updateImg(); }
}
}
