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

#include "Tags/CalibrationSet.hpp"

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
  bind->bindNew("filename", _fileName, RhIO::Bind::PullOnly)
    ->comment("where to save the values")
    ->persisted(false)
    ->defaultValue("arucoCalibration.csv");
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

  bind->bindNew("elbowOffset", _elbowOffset, RhIO::Bind::PullOnly)
    ->comment("Ensure elbows are not masking nearby tags")
    ->minimum(0)
    ->maximum(160)
    ->defaultValue(150);

  bind->bindNew("shoulderRollOffset", _shoulderRollOffset, RhIO::Bind::PullOnly)
    ->comment("Ensure that arms are not colliding with the trunk")
    ->minimum(0)
    ->maximum(20)
    ->defaultValue(15);

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

  bind->bindNew("smoothingTime", _smoothingTime, RhIO::Bind::PullOnly)
    ->comment("The transition at start/end of dance [s]")
    ->defaultValue(2);

  bind->bindNew("logDuration", _logDuration, RhIO::Bind::PullOnly)
    ->comment("How much time is used for the log? [s]")
    ->defaultValue(26);

  // Ensures default values are pushed
  bind->push();
}

std::string ArucoCalibration::getName() { return "arucoCalibration"; }

void ArucoCalibration::onStart() {
  _container.clear();
  bind->pull();
  _t = 0;
  logger.log("Starting ArucoCalibration");

  CalibrationSet cs;
  std::map<int,ArucoTag> tags = cs.getMarkers();

  // Creating an entry for each known id tag and retriving its position
  _container.clear();
  _mapOfTagPositions.clear();
  for (auto & pair : tags) {
    int index = pair.first;
    Eigen::Vector3d center= pair.second.marker_center;
    _mapOfTagPositions[index] = {center.x(), center.y(), center.z()};
    _container[index] = (std::vector<std::vector<double> >());
  }
}

// Fetching the low level info for the given timestamp and adding an entry for
// each detected tag
void ArucoCalibration::addEntry(std::vector<int> &indices,
                                std::vector<std::pair<float, float>> & centers,
                                std::vector<std::pair<float, float>> & uncorrected_centers) {
  // The imu value are not stored directly into the model as the DOF are. This
  // functions retrieves the
  // imu values. Leph has a test that proves that this works (the stoning should
  // be his when we
  // find out this doesn't work).
  Eigen::Vector3d RollPitchYaw = _model->trunkSelfOrientation();
  double pitch = RollPitchYaw[1];
  double roll = RollPitchYaw[0];
  for (unsigned int i = 0; i < indices.size(); i++) {
    // For each detected tag
    int indice = indices[i];
    std::pair<float, float> center = centers[i];
    std::pair<float, float> uncorrected_center = uncorrected_centers[i];
    // In Leph: pixels are in [-1, 1] and not [0, 1]
    double pixelX = (center.first - 0.5)*2;
    double pixelY = (center.second - 0.5)*2;
    double pixelXUncorrected = (uncorrected_center.first - 0.5)*2;
    double pixelYUncorrected = (uncorrected_center.second - 0.5)*2;
    
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
    data.push_back(pixelXUncorrected);
    data.push_back(pixelYUncorrected);
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
  file << "'pixel_y' ";
  file << "'pixel_x_uncorrected' ";
  file << "'pixel_y_uncorrected'\n";

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

  if (_t < 2 * _smoothingTime + _logDuration) {
    dance(_t);
    moveHead();
  }
  
  auto loc = getServices()->localisation;
  std::vector<int> markerIndices;
  std::vector<Eigen::Vector3d> markerPositions;
  std::vector<std::pair<float, float>> markerCenters;
  std::vector<std::pair<float, float>> markerUncorrectedCenters;
  double tagTimestamp = 0;
  // Getting the detected tags from the vision.
  // The thread safety is handled inside the function
#ifdef VISION_COMPONENT
  loc->stealTags(markerIndices, markerPositions, markerUncorrectedCenters, markerCenters, &tagTimestamp);
#endif
  if (_t < _smoothingTime || markerIndices.size() < 1) {
    // Either the vision didn't detect anything, or we're asking for the tag
    // info again or robot is not moving really yet
    // (this function will be ticked faster than the Vision)
    bind->push();
    return;
  }

  logger.log("Adding entry for TS %f", tagTimestamp);
  // Retrieving the model at the given timestamp
  _model->setAutoUpdate(true);
  Helpers::getServices()->model->pastReadModel(tagTimestamp, _pastReadModel);
  _model = &(_pastReadModel.get());
  _model->setAutoUpdate(false);
  _model->updateDOFPosition();

  // We have observations, let's log them along with the robot state
  addEntry(markerIndices, markerCenters, markerUncorrectedCenters);

  std::cout << "_t : " << _t << std::endl;

  if (_t > _logDuration + _smoothingTime * 2) {
    //Printing how often the tags where seen
    for(auto const &pair : _container) {
      int index = pair.first;
      int nbPresent = pair.second.size();
      logger.log("The key %d was present %d times", index, nbPresent);
    }
    // Our job here is done, saving the file
    writeFile();
    // End move
    this->Move::stop();
  }

  bind->push();
}


void ArucoCalibration::dance(float time) {
  float freq = _bpm/60.0;
  double gain = 1;
  double end_move = _smoothingTime + _logDuration;
  if (time < _smoothingTime) {
    gain = time / _smoothingTime;
  } else if (time > end_move) {
    gain = std::max(0.0, 1 - (time - end_move) / _smoothingTime);
  }
  // Oscillation on robot roll
  setAngle("left_hip_roll"   ,  gain * _losangeAmplitude*sin(2*M_PI*freq*time));
  setAngle("right_hip_roll"  ,  gain * _losangeAmplitude*sin(2*M_PI*freq*time));
  setAngle("left_ankle_roll" , -gain * _losangeAmplitude*sin(2*M_PI*freq*time));
  setAngle("right_ankle_roll", -gain * _losangeAmplitude*sin(2*M_PI*freq*time));
  // Oscillation on hip pitch
  setAngle("left_hip_pitch"  , gain * _hipPitchAmplitude*sin(2*M_PI*freq/2.0*time));
  setAngle("right_hip_pitch" , gain * _hipPitchAmplitude*sin(2*M_PI*freq/2.0*time));
  // Setting elbow straight
  setAngle("right_elbow", gain * _elbowOffset);
  setAngle("left_elbow" , gain * _elbowOffset);
  // Moving arms further from the body
  setAngle("left_shoulder_roll",  gain * _shoulderRollOffset);
  setAngle("right_shoulder_roll", -gain * _shoulderRollOffset);
}

void ArucoCalibration::stopDance() {
  RhIO::Root.setBool("/moves/walk/walkEnable", 0);
}


void ArucoCalibration::moveHead() {
  RhIO::Root.setFloat("/moves/head/localizeMaxPan", 120);
  RhIO::Root.setFloat("/moves/head/localizeMinOverlap", 20);
  RhIO::Root.setFloat("/moves/head/localizeMaxTilt", 90);
  RhIO::Root.setFloat("/moves/head/maxSpeed", _headSpeed);
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
