#include "Replayer.hpp"

#include "rhoban_utils/logging/logger.h"
#include "rhoban_utils/serialization/json_serializable.h"

using namespace rhoban_utils;

static Logger logger("Replayer");

//TODO : put dof groups shortcuts in a helper
std::vector<std::string> _dofNames = {
  "left_hip_pitch",
  "left_hip_roll",
  "left_knee",
  "left_ankle_pitch",
  "left_ankle_roll",
  "left_hip_yaw",
  "right_hip_yaw",
  "right_hip_pitch",
  "right_hip_roll",
  "right_knee",
  "right_ankle_pitch",
  "right_ankle_roll",
  "left_shoulder_pitch",
  "left_shoulder_roll",
  "left_elbow",
  "right_shoulder_pitch",
  "right_shoulder_roll",
  "right_elbow"
};

Replayer::Replayer() {
    Move::initializeBinding();
    time=0.0;
    bind->bindNew("speed", speed)
        ->defaultValue(5.0)->minimum(0.0)->maximum(50.0)
        ->comment("Speed factor for standing up")
        ->persisted(true);
    bind->bindNew("enable", enable)
        ->defaultValue(true)->persisted(true);
    bind->bindNew("over", over, RhIO::Bind::PushOnly)
        ->comment("Is the move over?")
        ->defaultValue(true);
}

Replayer::~Replayer() {
}

std::string Replayer::getName() {
    return "replayer";
}

void Replayer::onStart() {
    time = 0.0;
    over = false;

    try {
      splines = Function::fromFile("staticPositions.json");
    } catch (const rhoban_utils::JsonParsingError & exc) {
      logger.error("%s", exc.what());
    }
}

void Replayer::step(float elapsed) {
    bind->pull();
    
    if (enable) {
        time += elapsed*speed/5.0;
	//	std::cout << "time = " << time << std::endl;
	for (const auto& name : _dofNames) {
	  setAngle(name, splines[name].get(time));
	  if (name == "left_elbow") {
	    //  std::cout << "left_elbow angle = " << splines[name].get(time) << std::endl;
	  }
	}

        over = splines["over"].get(time) > 0.5;
    } else {
        over = true;
    }
}
