#include "ImageLogger.h"

#include <rhoban_utils/util.h>

#include <opencv2/highgui/highgui.hpp>

namespace Vision {
namespace Utils {

ImageLogger::ImageLogger(const std::string & logger_prefix, bool store_images, int max_img)
  : logger_prefix(logger_prefix),
    store_images(store_images),
    max_img(max_img)
{
}

bool ImageLogger::isActive() const {
  return session_path != "";
}

void ImageLogger::pushEntry(const rhoban_utils::TimeStamp & timestamp, const cv::Mat & img) {
  // Start session if required
  if (!isActive()) {
    initSession();
  }
  // If too much images have been written, close session and throw a runtime error
  if (img_index >= max_img)  {
    endSession();
    throw std::runtime_error(DEBUG_INFO + " max images reached");
  }
  // Store or write imaged depending on mode
  Entry entry(timestamp, img.clone());
  if (store_images) {
    entries_map[img_index] = entry;
  } else {
    writeEntry(img_index, entry);
  }
  img_index++;
}

void ImageLogger::endSession() {
  if (entries_map.size() != 0) {
    for (const auto & pair : entries_map) {
      writeEntry(pair.first, pair.second);
    }
  }
  description_file.close();
  session_path = "";
}

void ImageLogger::initSession(const std::string & session_local_path) {
  if (session_local_path != "") {
    session_path = logger_prefix + "/" + session_local_path;
  } else {
    // Use a default name
    session_path = logger_prefix + "/" + rhoban_utils::getFormattedTime();
  }
  int err = system(("mkdir -p " + session_path).c_str());
  if (err != 0) {
    throw std::runtime_error(DEBUG_INFO + "Failed to create dir: '" + session_path + "'");
  }
  std::string file_path = session_path + "/images.csv";
  description_file.open(file_path);
  if (!description_file.good()) {
    throw std::runtime_error(DEBUG_INFO + "Failed to open file: '" + file_path + "'");
  }
}

const std::string & ImageLogger::getSessionPath() {
  return session_path;
}


void ImageLogger::writeEntry(int idx, const Entry & e) {
  // Building image name
  int nb_digits = std::log10(max_img);
  int currentDigits = 1;
  if (idx > 0 ) {
    currentDigits = std::log10(idx);
  }
  std::ostringstream image_path_oss;
  for (int i = 0; i < nb_digits - currentDigits; i++) {
    image_path_oss << "0";
  }
  image_path_oss << idx << ".png";
  std::string img_name = image_path_oss.str();
  // Writing image
  cv::imwrite(session_path + "/" + img_name, e.second);
  // Writing couple timestamp_path to file
  description_file << (unsigned long)e.first.getTimeMS() << "," << img_name << std::endl;
}

}
}
