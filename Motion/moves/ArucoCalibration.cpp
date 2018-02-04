#include "moves/ArucoCalibration.h"
#include "Walk.h"
#include "rhoban_utils/logging/logger.h"
#include "scheduler/MoveScheduler.h"
#include "services/LocalisationService.h"
#include "services/ModelService.h"
#include "services/ModelService.h"
#include <Model/HumanoidFixedModel.hpp>
#include <Model/NamesModel.h>
#include <cmath>
#include <rhoban_utils/angle.h>
#include <string>

static rhoban_utils::Logger logger("ArucoCalibration");

// Visual verification
void showTagArray();

// All of the above should be done in the vision world.
// This move should only handle the motion inteligence and set a flag in
// LocalisationService when the positions are reached?

ArucoCalibration::ArucoCalibration()
    : Move(),
      _pastReadModel(InitHumanoidModel<Leph::HumanoidFixedPressureModel>()) {
  _model = &(_pastReadModel.get());
  Move::initializeBinding();
  bind->bindNew("nbFramesInLog", _nbFramesInLog, RhIO::Bind::PullOnly)
      ->comment("Number of frames that will be taken in the log")
      ->persisted(true)
      ->defaultValue(150)
      ->minimum(1)
      ->maximum(1000);
  bind->bindNew("currentNbFrames", _nbFramesRead, RhIO::Bind::PushOnly)
      ->comment("Current number of processed frames")
      ->defaultValue(0);
  bind->bindNew("filename", _fileName, RhIO::Bind::PullOnly)
      ->comment("where to save the values")
      ->persisted(false)
      ->defaultValue("arucoCalibration.csv");
  bind->bindNew("disabled", _disabled, RhIO::Bind::PushOnly)
      ->comment("Starts being false, becomes true when the measures are over.")
      ->defaultValue(false);
  bind->bindNew("dance", _dance, RhIO::Bind::PullOnly)
      ->comment("Starts a dance on the robot")
      ->persisted(true)
      ->defaultValue(1);
  bind->bindNew("bpm", _bpm, RhIO::Bind::PullOnly)
      ->comment("Sets the frequency of the dance")
      ->persisted(true)
      ->defaultValue(15)
      ->minimum(0.0)
      ->maximum(300.0);

    bind->bindNew("headSpeed", _headSpeed, RhIO::Bind::PullOnly)
      ->comment("Sets the angular speed in the head move")
      ->persisted(true)
      ->defaultValue(30)
      ->minimum(0.0)
      ->maximum(300.0);
 
    /*bind->bindNew("elbowOffset", _elbowOffset, RhIO::Bind::PullOnly)
      ->comment("Sets an offset to the elbow (negative values only, 0 is "
                "straight, actually changes the walk variable)")
      ->persisted(true)
      ->minimum(-180.0)
      ->maximum(0.0)
      ->defaultValue(-20.0);*/
  bind->bindNew("handTuned", _handTuned, RhIO::Bind::PullOnly)
      ->comment("If true, the head will be handled by hand, throught pan and tilt")
      ->persisted(true)
      ->defaultValue(0);
  bind->bindNew("pan", _pan, RhIO::Bind::PullOnly)
      ->comment("Hand tuned pan")
      ->minimum(-180.0)
      ->maximum(180.0)
      ->defaultValue(0);
  bind->bindNew("tilt", _tilt, RhIO::Bind::PullOnly)
      ->comment("Hand tuned tilt")
      ->minimum(-30)
      ->maximum(90)
      ->defaultValue(0);

  bind->bindNew("hipPitchAmplitude", _hipPitchAmplitude, RhIO::Bind::PullOnly)
      ->comment("Sets the amplitude of a sinus added to the hip pitchs")
      ->minimum(0)
      ->maximum(40)
      ->defaultValue(14);

    bind->bindNew("losangeAmplitude", _losangeAmplitude, RhIO::Bind::PullOnly)
      ->comment("Sets the amplitude of a sinus added to the hip+ankle losange")
      ->minimum(0)
      ->maximum(40)
      ->defaultValue(12);
    /*
  bind->bindNew("trunkXAmplitude", _trunkXAmplitude, RhIO::Bind::PullOnly)
      ->comment("Sets the amplitude of a sinus addedto trunkXAmplitude")
      ->minimum(0)
      ->maximum(0.1)
      ->defaultValue(0.04);
  bind->bindNew("trunkYAmplitude", _trunkYAmplitude, RhIO::Bind::PullOnly)
      ->comment("Sets the amplitude of a sinus addedto trunkYAmplitude")
      ->minimum(0)
      ->maximum(0.1)
      ->defaultValue(0.04);
  bind->bindNew("trunkZAmplitude", _trunkZAmplitude, RhIO::Bind::PullOnly)
      ->comment("Sets the amplitude of a sinus addedto trunkZAmplitude")
      ->minimum(0)
      ->maximum(0.1)
      ->defaultValue(0.02);
    */

  bind->bindNew("maxFramesPerTag", _maxFramesPerTag, RhIO::Bind::PullOnly)
    ->comment("Sets the maximum recorded lines for a given tag")
    ->minimum(1)
    ->maximum(100)
    ->defaultValue(20);
}

std::string ArucoCalibration::getName() { return "arucoCalibration"; }

void ArucoCalibration::onStart() {
  _container.clear();
  _nbFramesRead = 0;
  bind->pull();
  _t = 0;
  logger.log("Starting ArucoCalibration");
  _disabled = false;

  float x0 = 0.158;
  float y0 = 0.17;
  float z0 = 0.003;
  float deltaX = 0.510;
  float deltaY = 0.65;
  float absoluteBigX = 1.281;
  float bigDeltaZ = 0.1485;
  float superBigDeltaZ = 0.43;
  float superBigDeltaX = 0.025;

  //        [3][6]
  //         |
  //[1]-[2]-[4][7][9]
  //         |
  //        [5][8]
  //1
  _mapOfTagPositions[0] = std::vector<double>{x0, y0, z0};
  _mapOfTagPositions[1] = std::vector<double>{x0, -y0, z0};
  //2
  _mapOfTagPositions[2] = std::vector<double>{x0 + deltaX, y0, z0};
  _mapOfTagPositions[3] = std::vector<double>{x0 + deltaX, -y0, z0};

  //6
  _mapOfTagPositions[29] = std::vector<double>{absoluteBigX, deltaY, bigDeltaZ};
  //7
  _mapOfTagPositions[62] = std::vector<double>{absoluteBigX, 0.0, bigDeltaZ};
  //8
  _mapOfTagPositions[141] = std::vector<double>{absoluteBigX, -deltaY, bigDeltaZ};
  //9
  _mapOfTagPositions[173] = std::vector<double>{absoluteBigX + superBigDeltaX, 0.0, superBigDeltaZ};

  _container.clear();
  // Creating an entry for each known id tag
  for (const auto & pair : _mapOfTagPositions) {
    int index = pair.first;
    _container[index] = (std::vector<std::vector<double> >());
  }
  
  /*
  float leftFootX = 0.0544;
  float leftFootY = 0.075;
  float x0 = 0.22 - leftFootX;
  float y0Right = -0.17 - leftFootY;
  float y0Left = 0.17 - leftFootY;
  float z0 = 0.003;
  float deltaX = 0.46;
  float deltaY = 0.67;
  float absoluteBigX = 1.256;//0.07425;
  //float bigDeltaX = 0.338; // This works too, confirms 2*deltaX+bigDeltaX = absoluteBigX
  float bigDeltaZ = 0.1485;

  //        [3][6]
  //         |
  //[1]-[2]-[4][7]
  //         |
  //        [5][8]
  //1
  _mapOfTagPositions[0] = std::vector<double>{x0, y0Left, z0};
  _mapOfTagPositions[1] = std::vector<double>{x0, y0Right, z0};
  //2
  _mapOfTagPositions[2] = std::vector<double>{x0 + deltaX, y0Left, z0};
  _mapOfTagPositions[3] = std::vector<double>{x0 + deltaX, y0Right, z0};

  //3
  _mapOfTagPositions[4] = std::vector<double>{x0 + 2*deltaX, y0Left + deltaY, z0};
  _mapOfTagPositions[5] = std::vector<double>{x0 + 2*deltaX, y0Right + deltaY, z0};
  //4
  _mapOfTagPositions[6] = std::vector<double>{x0 + 2*deltaX, y0Left, z0};
  _mapOfTagPositions[7] = std::vector<double>{x0 + 2*deltaX, y0Right, z0};
  //5
  _mapOfTagPositions[8] = std::vector<double>{x0 + 2*deltaX, y0Left - deltaY, z0};
  _mapOfTagPositions[9] = std::vector<double>{x0 + 2*deltaX, y0Right - deltaY, z0};

  //6
  _mapOfTagPositions[29] = std::vector<double>{absoluteBigX - leftFootX, deltaY - leftFootY, bigDeltaZ}; // 0.338 + 2*deltax , deltaY, bigDeltaZ};
  //7
  _mapOfTagPositions[62] = std::vector<double>{absoluteBigX - leftFootX, - leftFootY, bigDeltaZ}; // // 0.338 + 2*deltax, deltaY, bigDeltaZ}
  //8
  _mapOfTagPositions[141] = std::vector<double>{absoluteBigX - leftFootX, -deltaY - leftFootY, bigDeltaZ}; // 0.338 + 2*deltax, , bigDeltaZ}
  */

}

// Fetching the low level info for the given timestamp and adding an entry for
// each detected tag
void ArucoCalibration::addEntry(std::vector<int> &indices,
                                std::vector<std::pair<float, float>> &centers) {
  // The imu value are not stored directly into the model as the DOF are. This
  // functions retrieves the
  // imu values. Leph has a test that proves that this works (the stoning should
  // be his when we
  // find out this doesn't work).
  Eigen::Vector3d RollPitchYaw = _model->trunkSelfOrientation();
  double pitch = RollPitchYaw[1];
  double roll = RollPitchYaw[0];
  double pixelX = 0.0;
  double pixelY = 0.0;
  for (unsigned int i = 0; i < indices.size(); i++) {
    // For each detected tag
    int indice = indices[i];
    if (_container[indice].size() >= (size_t)_maxFramesPerTag) {
      // We have enough of this tag
      continue;
    }
    std::pair<float, float> center = centers[i];
    // #LephConventionsAreGr8 pixels are in [-1, 1] and not [0, 1]
    pixelX = (center.first - 0.5)*2;
    pixelY = (center.second - 0.5)*2;
    
    if (!(_mapOfTagPositions.count(indice))) {
      logger.log("The index %d was seen but its position is unknown.", indice);
      continue;
    }
    std::vector<double> tagPosition = _mapOfTagPositions[indice];
    // Low level info

    std::vector<double> data;
    for (const std::string name : Leph::NamesDOF) {
      data.push_back(_model->getDOF(name));
    }
    data.push_back(pitch);
    data.push_back(roll);
    data.push_back(tagPosition[0]);
    data.push_back(tagPosition[1]);
    data.push_back(tagPosition[2]);
    data.push_back(pixelX);
    data.push_back(pixelY);
    _container[indice].push_back(data);
    data.clear();
  }
}

void ArucoCalibration::writeFile() {
  logger.log("################################# Writing calibration file at %s", _fileName.c_str());
  std::ofstream file;
  file.open(_fileName, std::ios_base::trunc);
  // Adding the header
  std::vector<std::string> data;
  file << "# ";
  for (const std::string name : Leph::NamesDOF) {
    file << "'" << name << "' ";
  }
  file << "'imu_pitch' ";
  file << "'imu_roll' ";
  file << "'ground_x' ";
  file << "'ground_y' ";
  file << "'ground_z' ";
  file << "'pixel_x' ";
  file << "'pixel_y'\n";

  // Adding the content
  for (auto & pair : _container) {
    // Adding all the lines for tag of index pair.first
    for (auto & line : pair.second) {
      for (auto & word : line) {
        file << word << " ";
      }
      file << "\n";
    }
  }

  file.close();
}

void ArucoCalibration::step(float elapsed) {
  bind->pull();
  _t = _t + elapsed;
  if (_disabled) {
    onStop();
    return;
  }

  // Setting the elbow straight
  //RhIO::Root.setFloat("/moves/walk/elbowOffset", _elbowOffset);
  if (_dance && !_handTuned) {
    float freq = _bpm/60.0;
    dance(freq, _t);
    moveHead(_headSpeed);
  } else {
    stopDance();
    stopHead();
  }

  if (_handTuned) {
    std::cout << "Hand tunned mode..." << std::endl;
    setAngle("head_yaw", _pan);
    setAngle("head_pitch", _tilt);
  }
  
  auto loc = getServices()->localisation;
  std::vector<int> markerIndices;
  std::vector<Eigen::Vector3d> markerPositions;
  std::vector<std::pair<float, float>> markerCenters;
  double tagTimestamp = 0;
  // Getting the detected tags from the vision.
  // The thread safety is handled inside the function
#ifdef VISION_COMPONENT
  loc->stealTags(markerIndices, markerPositions, markerCenters, &tagTimestamp);
#endif
  if (markerIndices.size() < 1) {
    // Either the vision didn't detect anything, or we're asking for the tag
    // info again
    // (this function will be ticked faster than the Vision)
    bind->push();
    return;
  }
  _nbFramesRead = _nbFramesRead + 1;
  logger.log("Adding entry for TS %f", tagTimestamp);
  // Retrieving the model at the given timestamp
  _model->setAutoUpdate(true);
  Helpers::getServices()->model->pastReadModel(tagTimestamp, _pastReadModel);
  _model = &(_pastReadModel.get());
  _model->setAutoUpdate(false);
  _model->updateDOFPosition();

  // We have observations, let's log them along with the robot state
  addEntry(markerIndices, markerCenters);

  // Checking if we have enough frames for each tag
  bool weAreDone = true;
  for (const auto & pair : _container) {
    if (pair.second.size() != (size_t)_maxFramesPerTag) {
      weAreDone = false;
      break;
    }
  }

  if ((_nbFramesRead >= _nbFramesInLog) || weAreDone) {
    _disabled = true;
    _nbFramesRead = 0;
    //Printing how often the tags where seen
    for(auto const &paire : _container) {
      int index = paire.first;
      int nbPresent = paire.second.size();
      logger.log("The key %d was present %d times", index, nbPresent);
    }

    // Making sure the seen tags are evenly represented by repeating inputs from underrepresented tags
    std::cout << "Equalizing..." << std::endl;
    equalizeTags();
    std::cout << "Tags equalized : " << std::endl;
    for(auto const &paire : _container) {
      int index = paire.first;
      int nbPresent = paire.second.size();
      logger.log("The key %d was present %d times", index, nbPresent);
    }
    // Our job here is done, saving the file
    writeFile();
  }

  bind->push();
}


// Special Steve dédicace
void ArucoCalibration::dance(float freq, float time) {
  /*
  RhIO::Root.setBool("/moves/walk/walkEnable", 1);
  RhIO::Root.setFloat("/moves/walk/trunkXOffset_backward", _trunkXAmplitude*sin(2*M_PI*freq*time)); // 0.04
  RhIO::Root.setFloat("/moves/walk/trunkXOffset_forward", _trunkXAmplitude*sin(2*M_PI*freq*time)); // 0.04
  RhIO::Root.setFloat("/moves/walk/trunkYOffset", -_trunkYAmplitude*cos(2*M_PI*freq*time));
  RhIO::Root.setFloat("/moves/walk/trunkZOffset", 0.04 + _trunkZAmplitude*sin(2*M_PI*freq*time));
  */

  setAngle("left_hip_roll", _losangeAmplitude*sin(2*M_PI*freq*time));
  setAngle("right_hip_roll", _losangeAmplitude*sin(2*M_PI*freq*time));
  setAngle("left_ankle_roll", -_losangeAmplitude*sin(2*M_PI*freq*time));
  setAngle("right_ankle_roll", -_losangeAmplitude*sin(2*M_PI*freq*time));
  
  //trunkzoffset 0.05 max
  // There is no pitchSwing. Trynig a different freq
  setAngle("left_hip_pitch", _hipPitchAmplitude*sin(2*M_PI*freq/2.0*time));
  setAngle("right_hip_pitch", _hipPitchAmplitude*sin(2*M_PI*freq/2.0*time));
}

void ArucoCalibration::stopDance() {
  RhIO::Root.setBool("/moves/walk/walkEnable", 0);
}


void ArucoCalibration::moveHead(float angularSpeed) {
  RhIO::Root.setFloat("/moves/head/localizeMaxPan", 80);
  RhIO::Root.setFloat("/moves/head/localizeMaxTilt", 90);
  RhIO::Root.setFloat("/moves/head/maxSpeed", angularSpeed);
  RhIO::Root.setBool("/moves/head/disabled", 0);
  RhIO::Root.setBool("/moves/head/forceLocalize", 1);
}

void ArucoCalibration::stopHead() {
  RhIO::Root.setBool("/moves/head/disabled", 1);
}

void ArucoCalibration::onStop() {
  stopDance();
  stopHead();
}

void ArucoCalibration::equalizeTags() {
  if (_container.size() < 1) {
    return;
  }
  unsigned int max = 0;
  for(auto const &paire : _container) {
    if (paire.second.size() > max) {
      // paire.second.size() is the number of lines for paire.first id
      max = paire.second.size();
    }
  }
  
  // Convention : every tag will have 'max' appearences
  for(auto const &paire : _container) {
    int index = paire.first;
    std::cout << "index " << index << " nb : " << paire.second.size() << std::endl; 
    if (paire.second.size() >= max || paire.second.size() == 0) {
      // We either have enough lines or we don't have any and there is no point in trying
      continue;
    }
    int nbToBeAdded = max - paire.second.size(); 
    int nbAdded = 0;
    std::vector<std::vector<double>> extra;
    while(nbAdded < nbToBeAdded) {
      for (std::vector<double> & list : _container[index]) {
        extra.push_back(list);
        nbAdded++;
        if (nbAdded >= nbToBeAdded) {
          break;
        }
      }
    }
    _container[index].insert(_container[index].end(), extra.begin(), extra.end());
  }

}
 
