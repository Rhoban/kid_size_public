#pragma once

#include <map>
#include <functional>

#include "Filters/Pipeline.hpp"

namespace Vision {
namespace Application {
class Application : public rhoban_utils::JsonSerializable {
public:
  Application();
  virtual ~Application();

  virtual void configure(int argc, char *argv[]);
  virtual void launch();

  /// Return false if application has finished
  bool isActive() const;

  virtual void printHelp();

  // Json stuff
  virtual void fromJson(const Json::Value & v, const std::string & dir_name) override;
  virtual Json::Value toJson() const override;
  virtual std::string getClassName() const override { return "Application"; }

  /// Throws a logic_error if content is inconsistent (replay vs realtime)
  void checkConsistency() const;

protected:
  // Pair of callback and comment
  typedef std::pair<std::function<void()>, std::string> KeyAction;

  Pipeline pipeline;
  std::map<char, KeyAction> keyBindings;

  virtual void init();
  virtual void step();
  virtual void finish();

  Filter::UpdateType updateType;
  bool playNext;
  bool playing;
  bool embedded; // No waitkey allowed on embedded version
  bool gpuOn;
  bool end;
  std::string pathToLog;
  float angularPitchTolerance = 0.0;

  Utils::CameraState *lastCS, *cs;
  mutable std::mutex csMutex;

  bool exit_on_stream_end;
};
}
}
