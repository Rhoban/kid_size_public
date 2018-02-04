#include "Filters/Source/Source.hpp"

#include "Application.hpp"

#include <rhoban_utils/logging/logger.h>

#include <opencv2/highgui/highgui.hpp>

#include <exception>
#include <iostream>
#include <sstream>

using Vision::Filters::Source;

static rhoban_utils::Logger logger("Vision::Application");


namespace Vision {
namespace Application {


Application::Application()
      : updateType(Filter::UpdateType::forward),
        playing(false), embedded(false), gpuOn(false),
        pathToLog("") {
}

Application::~Application() {}

void Application::configure(int argc, char *argv[]) {
  if (argc < 2) {
    std::ostringstream oss;
    oss << "Usage: " << argv[0] << " <config_file>";
    throw std::runtime_error(oss.str());
  }
  loadFile(argv[1]);
}

void Application::init() {
  playNext = true;
  end = false;
  keyBindings[' '] = KeyAction([this]() { this->playing = !this->playing; },
                               "Start/Stop playing the video at full speed");
  keyBindings['n'] = KeyAction([this]() {
    this->playNext = true;
    this->playing = false;
    this->updateType = Filter::UpdateType::forward;
  }, "Next image (stop playing if activated)");
  keyBindings['h'] = KeyAction([this]() { this->printHelp(); },
                               "Display available commands along with help");
  // keyBindings['s'] = KeyAction([this](){this->save_file();},
  //                             "Save the pipeline to json file");
  keyBindings['q'] =
      KeyAction([this]() { this->end = true; }, "Quit the application");
  keyBindings['p'] = KeyAction([this]() {
    this->playNext = true;
    this->playing = false;
    this->updateType = Filter::UpdateType::backward;
  }, "Previous image (stop playing if activated)");
  keyBindings['u'] = KeyAction([this]() {
    this->playNext = true;
    this->playing = false;
    this->updateType = Filter::UpdateType::steady;
  }, "Update image (stop playing if activated)");
}

void Application::step() {
  if (!pipeline.step(updateType)) {
    std::cout << "pipeline.step() returned false, time to leave" << std::endl;
    end = true;
  }
}

void Application::finish() {}

void Application::printHelp() {
  for (const auto &action : keyBindings) {
    std::cout << "'" << action.first << "':\t" << action.second.second
              << std::endl;
  }
}

void Application::launch() {
  init();
  while (!end) {
    int64 start, stop;
    double elapsedTime(0);
    if (playing || playNext) {
      start = cv::getTickCount();
      step();
      stop = cv::getTickCount();
      // elapsed time in ms
      elapsedTime = 1000 * (stop - start) / cv::getTickFrequency();
      playNext = false;
    }
    int key(-1);
    logger.log("embedded = %d", embedded);
    if (!embedded) {
      if (!playing) {
        key = cv::waitKey();
      } else {
        // TODO : are this magic numbers important?
        int timeToWait = (int)(33 - elapsedTime);
        int minWait = 5;
        if (timeToWait < minWait)
          timeToWait = minWait;
        logger.log("waitKey called in Application for %d (ms) : ", timeToWait);
        key = cv::waitKey(timeToWait);
        // key = cv::waitKey(0);
      }
    }

    if (key != -1) {
      try {
        keyBindings.at((char)key).first();
      } catch (const std::out_of_range &o) {
        printHelp();
      }
    }
  }
  finish();
}

void Application::fromJson(const Json::Value & v, const std::string & dir_name) {
  rhoban_utils::tryRead(v,"playing",&playing);
  rhoban_utils::tryRead(v,"embedded",&embedded);
  rhoban_utils::tryRead(v,"gpuOn",&gpuOn);
  rhoban_utils::tryRead(v,"pathToLog",&pathToLog);
  rhoban_utils::tryRead(v,"angularPitchTolerance",&angularPitchTolerance);

  pipeline.tryRead(v, "pipeline", dir_name);
  checkConsistency();
}

Json::Value Application::toJson() const {
  Json::Value v;
  v["playing"] = playing;
  v["embedded"] = embedded;
  v["gpuOn"] = gpuOn;
  v["angularPitchTolerance"] = angularPitchTolerance;
  v["pathToLog"] = pathToLog;
  v["pipeline"] = pipeline.toJson();
  return v;
}

void Application::checkConsistency() const {
  Source::Type expected_type =
    pathToLog == "" ? Source::Type::Online : Source::Type::Log;
  std::string expected_type_name =
    pathToLog == "" ? "Online" : "Log";
  std::vector<std::string> invalid_filters;
  for (const auto & entry : pipeline.filters()) {
    Source * source = dynamic_cast<Source *>(entry.second);
    if (source != nullptr) {
      if (source->getType() != expected_type) {
        invalid_filters.push_back(entry.first);
      }
    }
  }
  if (invalid_filters.size() != 0) {
    std::ostringstream oss;
    oss << "Application::checkConsistency: "
        << "Invalid filters found, expected type was: '"
        << expected_type_name << "', ";
    oss << "filters: ";
    for (const std::string & name : invalid_filters) {
      oss << "(name='" << name << "',type='" << pipeline.get(name).getClassName() << "')" << std::endl;
    }
    throw std::logic_error(oss.str());
  }
}

}
}
