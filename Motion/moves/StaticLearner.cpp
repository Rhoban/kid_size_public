#include <math.h>
#include "StaticLearner.hpp"
#include <RhIO.hpp>
#include <rhoban_utils/angle.h>
#include <Model/HumanoidFloatingModel.hpp>
#include <rhoban_utils/control/variation_bound.h>
#include "services/ModelService.h"
#include <cstring>
#include <RhAL.hpp>
#include "scheduler/MoveScheduler.h"
#include <fstream>
#include <string>
#include <sstream>
#include <vector>
#include "rhoban_utils/util.h"

#define ISMX true
#ifndef ISMX
#define MOTOR RhAL::RX
#else
#define MOTOR RhAL::MX
#endif

/*
 * TODO
 * - Change bools to ints so they are tune-able
 * - bug when r=0->r=1->r=0?
 * - Change default PID values for MX case
 * - Add pid configuration in the replayer (should be saved in the spline btw)
 */

std::vector<std::string> split(const std::string &s, char delim) {
  std::vector<std::string> elems;
  rhoban_utils::split(s, delim, elems);
  return elems;
}

//TODO check that all torques are at 100%
StaticLearner::StaticLearner() {
    Move::initializeBinding();

    bind->bindNew("pLegs", _pLegs, RhIO::Bind::PullOnly)
      ->comment("P for leg motors")->persisted(true)
      ->defaultValue(2)->minimum(0)->maximum(100);
    
    bind->bindNew("pArms", _pArms, RhIO::Bind::PullOnly)
      ->comment("P for leg motors")->persisted(true)
      ->defaultValue(6)->minimum(0)->maximum(100);

    bind->bindNew("iLegs", _iLegs, RhIO::Bind::PullOnly)
      ->comment("I for leg motors")->persisted(true)
      ->defaultValue(0)->minimum(0)->maximum(100);
    
    bind->bindNew("iArms", _iArms, RhIO::Bind::PullOnly)
      ->comment("I for leg motors")->persisted(true)
      ->defaultValue(0)->minimum(0)->maximum(100);

    bind->bindNew("s", _staticOn, RhIO::Bind::PullOnly)
      ->comment("If on, the robot will stop moving. If off, the robot will be movable but still hard.")->persisted(true)
      ->defaultValue(1)->minimum(0)->maximum(1);

    bind->bindNew("a", _staticOnArms, RhIO::Bind::PullOnly)
      ->comment("Like the s command but only for arms")->persisted(true)
      ->defaultValue(1)->minimum(0)->maximum(1);

    bind->bindNew("l", _staticOnLeftLeg, RhIO::Bind::PullOnly)
      ->comment("Like the s command but only for the left leg")->persisted(true)
      ->defaultValue(1)->minimum(0)->maximum(1);

    bind->bindNew("r", _staticOnRightLeg, RhIO::Bind::PullOnly)
      ->comment("Like the s command but only for the right leg")->persisted(true)
      ->defaultValue(1)->minimum(0)->maximum(1);
    
    bind->bindNew("i", _staticPointIndex, RhIO::Bind::PullOnly)
      ->comment("Increment this index to take a snapshot")->persisted(true)
      ->defaultValue(0)->minimum(0)->maximum(100);


    bind->bindNew("snapshot", _snapshot)
      ->comment("Sete to true to take a snapshot at the current index")->persisted(true)
      ->defaultValue(0)->minimum(0)->maximum(1);

    bind->bindNew("goToOn", _goToOn, RhIO::Bind::PullOnly)
      ->comment("If on, the robot will go to the recorded position of index 'p'")->persisted(true)
      ->defaultValue(0)->minimum(0)->maximum(1);

    bind->bindNew("p", _goToIndex)
      ->comment("Index of the position the robot shall go to is goToOn is true")->persisted(true)
      ->defaultValue(0)->minimum(0)->maximum(10000);

}

std::string StaticLearner::getName() {
    return "staticLearner";
}

void StaticLearner::onStart() {
    bind->pull();
    _oldPointIndex = 0;
    //Reducing the risks for the head in ccase of a fall
#ifndef ISMX
    getScheduler()->getManager()->dev<MOTOR>("head_yaw").complianceMarginCW() = 7;
    getScheduler()->getManager()->dev<MOTOR>("head_yaw").complianceMarginCCW() = 7;
    getScheduler()->getManager()->dev<MOTOR>("head_pitch").complianceMarginCW() = 7;
    getScheduler()->getManager()->dev<MOTOR>("head_pitch").complianceMarginCCW() = 7;
#else
    getScheduler()->getManager()->dev<MOTOR>("head_yaw").PGain() = 1;
    getScheduler()->getManager()->dev<MOTOR>("head_pitch").PGain() = 1;
#endif
    
    _dofNames = {
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

    _legsDofNames = {
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
    };

    _leftLegDofNames = {
      "left_hip_pitch",
      "left_hip_roll",
      "left_knee",
      "left_ankle_pitch",
      "left_ankle_roll",
      "left_hip_yaw"
    };

    _rightLegDofNames = {
      "right_hip_yaw",
      "right_hip_pitch",
      "right_hip_roll",
      "right_knee",
      "right_ankle_pitch",
      "right_ankle_roll"
    };

    _armsDofNames = {
      "left_shoulder_pitch",
      "left_shoulder_roll",
      "left_elbow",
      "right_shoulder_pitch",
      "right_shoulder_roll",
      "right_elbow"
    };

    /*for (auto name : _dofNames) {
      _mapOfPositions[name] = std::vector<float>();
      }*/

    for (auto name : _dofNames) {
      _mapOfExtraAngles[name] = 0.0;

      std::cout << name << " (torque) = " << getScheduler()->getManager()->dev<MOTOR>(name).torqueLimit().readValue().value
      	<< " and " << getScheduler()->getManager()->dev<MOTOR>(name).maxTorque() << std::endl;

      _mapOfActualGoalPosition[name] = getScheduler()->getManager()->dev<MOTOR>(name).goalPosition();
    }
    _oldStaticOn = _staticOn;
    
}
        
void StaticLearner::onStop() {
  savePositionsToFile("staticPositions.csv", _mapOfPositions);
  savePositionsToFileJson("staticPositions.json", _mapOfPositions);
}


void StaticLearner::step(float elapsed) {
  // Elapsed is not used in this movement
  (void) elapsed;

    bind->pull();
    if (_staticOn && !_oldStaticOn) {
      // When going back to the static mode, the actual goal positions must be updated
      for (auto name : _dofNames) {
	_mapOfActualGoalPosition[name] = getScheduler()->getManager()->dev<MOTOR>(name).goalPosition();
      }
    } else if (!_staticOn && _oldStaticOn) {
      // When leaving the static mode, the I sums are reseted to avoid annoying stuff
      for (auto name : _dofNames) {
	_mapOfExtraAngles[name] = 0.0;
      }
    }
    _oldStaticOn = _staticOn;
    
    // Updating Is
    for (auto name : _legsDofNames) {
      _mapOfIs[name] = _iLegs;
    }
    for (auto name : _armsDofNames) {
      _mapOfIs[name] = _iArms;
    }

    calculateExtraAngles();
    setPLegs();
    setPArms();
    

    if (_staticPointIndex > _oldPointIndex || _snapshot) {
      if (_snapshot) {
	savePositions(_goToIndex);
      } else {
	savePositions();
      }

      _snapshot = false;
      _oldPointIndex = _staticPointIndex;
      _goToIndex++;
    }

    if (_goToOn) {
      // In this mode, the robot goes to a recorded position
      goToPositions(_goToIndex);
    } else {
      if (!_staticOn) {
	// Goal angle = read angle
	followPresentPositions();
      } else {
	// /!\ since a, l and r were introduced, addExtraAngles should be rewritten accordingly to avoid the double setAngle of death. It didn't work well so not worth it.
	//Goal angle = goal angle + I offset
	//addExtraAngles();
	if (!_staticOnArms) {
	  followPresentPositionsArms();
	}
	if (!_staticOnLeftLeg) {
	  followPresentPositionsLeftLeg();
	}
	if (!_staticOnRightLeg) {
	  followPresentPositionsRightLeg();
	}
      }
    }
    
    bind->push();
}

void StaticLearner::savePositions(int position) {
  std::cout << "Saving positions for point " << _staticPointIndex << std::endl;
  for (auto name : _dofNames) {
    std::cout << name << " : " << getAngle(name) << std::endl;
    if (position == -1) {
      _mapOfPositions[name].push_back(getAngle(name));
    } else {
      _mapOfPositions[name].insert(_mapOfPositions[name].begin()+position, getAngle(name));
    }
  }
}

void StaticLearner::followPresentPositions() {
  for (auto name : _dofNames) {
    setAngle(name, getAngle(name) + _mapOfExtraAngles[name]);
  }
}

void StaticLearner::followPresentPositionsArms() {
  for (auto name : _armsDofNames) {
    setAngle(name, getAngle(name) + _mapOfExtraAngles[name]);
  }
}

void StaticLearner::followPresentPositionsLeftLeg() {
  for (auto name : _leftLegDofNames) {
    setAngle(name, getAngle(name) + _mapOfExtraAngles[name]);
  }
}

void StaticLearner::followPresentPositionsRightLeg() {
  for (auto name : _rightLegDofNames) {
    setAngle(name, getAngle(name) + _mapOfExtraAngles[name]);
  }
}

void StaticLearner::setPLegs() {
  for (auto name : _legsDofNames) {
#ifndef ISMX
    getScheduler()->getManager()->dev<MOTOR>(name).complianceMarginCW() = _pLegs;
    getScheduler()->getManager()->dev<MOTOR>(name).complianceMarginCCW() = _pLegs;
#else
    getScheduler()->getManager()->dev<MOTOR>(name).PGain() = _pLegs;
    getScheduler()->getManager()->dev<MOTOR>(name).PGain() = _pLegs;
    getScheduler()->getManager()->dev<MOTOR>(name).IGain() = _iLegs;
    getScheduler()->getManager()->dev<MOTOR>(name).IGain() = _iLegs;
#endif
  }
}

void StaticLearner::setPArms() {
  for (auto name : _armsDofNames) {
#ifndef ISMX
    getScheduler()->getManager()->dev<MOTOR>(name).complianceMarginCW() = _pArms;
    getScheduler()->getManager()->dev<MOTOR>(name).complianceMarginCCW() = _pArms;
#else
    getScheduler()->getManager()->dev<MOTOR>(name).PGain() = _pArms;
    getScheduler()->getManager()->dev<MOTOR>(name).PGain() = _pArms;
    getScheduler()->getManager()->dev<MOTOR>(name).IGain() = _iArms;
    getScheduler()->getManager()->dev<MOTOR>(name).IGain() = _iArms;
#endif
  }
}

void StaticLearner::goToPositions(int index) {
  try {
    for (auto name : _dofNames) {
      setAngle(name, _mapOfPositions[name][index]);
    }
  } catch (...) {
    throw std::runtime_error("Failed to set positions in goToPositions");
  }
}

void StaticLearner::savePositionsToFile(std::string fileName, std::map<std::string, std::vector<float>>& map) {
  std::ofstream file;
  file.open (fileName, std::ios_base::trunc);
  for (auto const& e : map) {
    //DOF name
    file << e.first << " ";
    for (auto const& value : e.second) {
      //List of positions
      file << value << " ";
    }
    file << "\n";
  }
  file.close();
}

void StaticLearner::savePositionsToFileJson(std::string fileName, std::map<std::string, std::vector<float>>& map) {
  std::ofstream file;
  std::string tab = "    ";
  int nbElementsInMap = map.size();
  file.open (fileName, std::ios_base::trunc);
  file << "{\n";
  int dofNumber = 0;
  int time = 0;
  for (auto const& e : map) {
    //DOF name
    file << tab << "\"" << e.first << "\"" << ": [\n";
    int nbElements = e.second.size();
    time = 0;
    for (auto const& value : e.second) {
      //List of positions
      file << tab << tab << "[" << time << ", " << value << "]";
      if (time == (nbElements-1)) {
	file << "\n";
      } else {
	file << ",\n";
      }
      time++;
    }
    if (dofNumber == (nbElementsInMap-1)) {
      file << tab << "],\n\n";
      // Adding the "over" field
      file << tab << "\"" << "over" << "\"" << ": [\n";
      file << tab << tab << "[0, 0],\n";
      file << tab << tab << "[" << time << ", 0],\n";
      float end = time + 0.5;
      file << tab << tab << "[" << end << ", " << 1 << "]\n";
      file << "]\n";
    } else {
      file << tab << "],\n";
    }

    dofNumber++;
  }
  file << "}";
  file.close();
}


// 0% security in this function though
void StaticLearner::readPositionsFromFile(std::string fileName, std::map<std::string, std::vector<float>>& map) {
  std::ifstream file(fileName);
  std::string name = "SomethingWentWrong";
  for( std::string line; getline( file, line ); ) {
    std::vector<std::string> words = split(line, ' ');
    bool first = true;
    for (std::string word : words) {
      if (word.length() < 1) {
	continue;
      } else {
	if (first) {
	  first = false;
	  name = word;
	} else {
	  map[name].push_back(stof(word));
	}

      }
    }
  }
  file.close();
}



void StaticLearner::calculateExtraAngles() {
  // Awfull way to compensate the lack of Integral control in the RX series
  float maxSum = 10.0;
  for (auto name : _dofNames) {
    float goalPos = 0;
    if (_staticOn) {
      goalPos = _mapOfActualGoalPosition[name];
    } else {
      goalPos = getScheduler()->getManager()->dev<MOTOR>(name).goalPosition();
    }
    float delta = goalPos - getAngle(name);

    _mapOfExtraAngles[name] = _mapOfExtraAngles[name] + delta * _mapOfIs[name]/10000.0;
    
    //Limiting the effects
    if (_mapOfExtraAngles[name] > maxSum) {
      _mapOfExtraAngles[name] = maxSum;
    } else if (_mapOfExtraAngles[name] < -maxSum) {
      _mapOfExtraAngles[name] = -maxSum;
    }
    /*
     * Note that this could be dangerous when playing around angle limits...
     */
    /*if (name == "left_elbow") {
      std::cout << "Goalpos = " << goalPos << ", delta = " << delta << ", extraAngle = " << _mapOfExtraAngles[name] << std::endl;
      }*/
  }
}

void StaticLearner::addExtraAngles() {
  //This seems to create unstable behaviours. Check waht goalPosition() actuelly answers.
  for (auto name : _dofNames) {
    setAngle(name, _mapOfExtraAngles[name] + getScheduler()->getManager()->dev<MOTOR>(name).goalPosition());
  }
}
