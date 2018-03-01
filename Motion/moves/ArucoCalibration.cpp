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
      ->defaultValue(500)
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

  bind->bindNew("maxFramesPerTag", _maxFramesPerTag, RhIO::Bind::PullOnly)
    ->comment("Sets the maximum recorded lines for a given tag")
    ->minimum(1)
    ->maximum(500)
    ->persisted(true)
    ->defaultValue(100);
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
  float maxDeltaY = 0.92;
  float minDeltaX = 0.045;

  // rectangle :
  // [*]
  float rectangleX = 0.2995;
  float rectangleY = 0.4395;

  // distance between two rectangles (it is the same in X and Y). Symbols - and |
  float edge = 0.21;

  // position of little arucos from the origin of its rectangle
  float littleArucoX = rectangleX-0.1/2;
  float littleArucoY = (rectangleY-0.1)/2;

  // distance to panel (it is the same in X and Y). Symbols : and ..
  float spaceToPanel = 0.053;

  // position of big aruco, in front of the robot, from the origin of its rectangle
  float frontBigArucoX = rectangleX + spaceToPanel;
  float bigArucoZ = 0.153;

  // position of sides big aruco from the origin of its rectangle
  float sideBigArucoX = rectangleX/2;
  float sideBigArucoY = rectangleY/2 + spaceToPanel;

  // Robot left foot offset
  double offsetx=0.094;
  double offsety=0.08;

  //   x
  // y_|
  //   . Z
  //
  //        103  104   105
  //         :    :     :
  //  102 ..[0]-[1,4 ]-[5]..106
  //         |    |     |
  //  101 ..[6]-[2,3 ]-[7]..107
  //         |    |     |
  //  100 ..[8]-[9,10]-[11]..108
  //              ↑
  //            robot

  _mapOfTagPositions[0] = std::vector<double>{littleArucoX+2*(rectangleX+edge), rectangleY+edge-littleArucoY, 0.003};
  _mapOfTagPositions[1] = std::vector<double>{littleArucoX+2*(rectangleX+edge), littleArucoY, 0.003};
  _mapOfTagPositions[4] = std::vector<double>{littleArucoX+2*(rectangleX+edge), -littleArucoY, 0.003};
  _mapOfTagPositions[5] = std::vector<double>{littleArucoX+2*(rectangleX+edge), -(rectangleY+edge)+littleArucoY, 0.003};

  _mapOfTagPositions[6] = std::vector<double>{littleArucoX+(rectangleX+edge), rectangleY+edge-littleArucoY, 0.003};
  _mapOfTagPositions[2] = std::vector<double>{littleArucoX+(rectangleX+edge), littleArucoY, 0.003};
  _mapOfTagPositions[3] = std::vector<double>{littleArucoX+(rectangleX+edge), -littleArucoY, 0.003};
  _mapOfTagPositions[7] = std::vector<double>{littleArucoX+(rectangleX+edge), -(rectangleY+edge)+littleArucoY, 0.003};

  _mapOfTagPositions[8] = std::vector<double>{littleArucoX, rectangleY+edge-littleArucoY, 0.003};
  _mapOfTagPositions[9] = std::vector<double>{littleArucoX, littleArucoY, 0.003};
  _mapOfTagPositions[10] = std::vector<double>{littleArucoX, -littleArucoY, 0.003};
  _mapOfTagPositions[11] = std::vector<double>{littleArucoX, -(rectangleY+edge)+littleArucoY, 0.003};

  _mapOfTagPositions[100] = std::vector<double>{sideBigArucoX, rectangleY+edge+sideBigArucoY, bigArucoZ};
  _mapOfTagPositions[101] = std::vector<double>{rectangleX+edge+sideBigArucoX, rectangleY+edge+sideBigArucoY, bigArucoZ};
  _mapOfTagPositions[102] = std::vector<double>{2*(rectangleX+edge)+sideBigArucoX, rectangleY+edge+sideBigArucoY, bigArucoZ};

  _mapOfTagPositions[103] = std::vector<double>{2*(rectangleX+edge)+frontBigArucoX, rectangleY+edge, bigArucoZ};
  _mapOfTagPositions[104] = std::vector<double>{2*(rectangleX+edge)+frontBigArucoX, 0, bigArucoZ};
  _mapOfTagPositions[105] = std::vector<double>{2*(rectangleX+edge)+frontBigArucoX, -(rectangleY+edge), bigArucoZ};

  _mapOfTagPositions[106] = std::vector<double>{2*(rectangleX+edge)+sideBigArucoX, -(rectangleY+edge+sideBigArucoY), bigArucoZ};
  _mapOfTagPositions[107] = std::vector<double>{rectangleX+edge+sideBigArucoX, -(rectangleY+edge+sideBigArucoY), bigArucoZ};


  /*
  //1: ids{0,1}
  _mapOfTagPositions[0] = std::vector<double>{x0, y0, z0};
  _mapOfTagPositions[1] = std::vector<double>{x0, -y0, z0};
  //2: ids{2,3}
  _mapOfTagPositions[2] = std::vector<double>{x0 + deltaX, y0, z0};
  _mapOfTagPositions[3] = std::vector<double>{x0 + deltaX, -y0, z0};
  //3: ids{29}
  _mapOfTagPositions[29] = std::vector<double>{absoluteBigX, deltaY, bigDeltaZ};
  //4: ids{62}
  _mapOfTagPositions[62] = std::vector<double>{absoluteBigX, 0.0, bigDeltaZ};
  //5: ids{141}
  _mapOfTagPositions[141] = std::vector<double>{absoluteBigX, -deltaY, bigDeltaZ};
  //6: ids{132}
  _mapOfTagPositions[132] = std::vector<double>{minDeltaX+deltaX, -maxDeltaY, bigDeltaZ};
  //7: ids{55}
  _mapOfTagPositions[55] = std::vector<double>{minDeltaX, -maxDeltaY, bigDeltaZ};
  */

  _container.clear();

  // Creating an entry for each known id tag
  for (auto & pair : _mapOfTagPositions) {
    int index = pair.first;
    pair.second[0]-=offsetx;
    pair.second[1]-=offsety;
    _container[index] = (std::vector<std::vector<double> >());
  }
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
  file << "# 'tag_id' ";
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
    int tag_id = pair.first;
    // Adding all the lines for tag of index pair.first
    for (auto & line : pair.second) {
      file << tag_id << " ";
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
  if (_t < 1.0 || markerIndices.size() < 1) {
    // Either the vision didn't detect anything, or we're asking for the tag
    // info again or robot is not moving really yet
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
    for(auto const &pair : _container) {
      int index = pair.first;
      int nbPresent = pair.second.size();
      logger.log("The key %d was present %d times", index, nbPresent);
    }

    for(auto const &pair : _container) {
      int index = pair.first;
      int nbPresent = pair.second.size();
      logger.log("The key %d was present %d times", index, nbPresent);
    }
    // Our job here is done, saving the file
    writeFile();
  }

  bind->push();
}


void ArucoCalibration::dance(float freq, float time) {

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
  RhIO::Root.setFloat("/moves/head/localizeMaxPan", 100);
  RhIO::Root.setFloat("/moves/head/localizeMinOverlap", 20);
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
