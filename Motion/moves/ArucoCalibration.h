#pragma once

#include "Move.h"
#include <Eigen/Dense>
#include <Model/HumanoidFixedPressureModel.hpp>
#include <Model/HumanoidModel.hpp>
#include <mutex>
#include <string>

class ArucoCalibration : public Move {
public:
  /**
   * Initialization
   */
  ArucoCalibration();

  /**
   * Implement Move
   */
  virtual std::string getName() override;
  virtual void onStart() override;
  virtual void step(float elapsed) override;
  virtual void onStop() override;
  
  void addEntry(std::vector<int> &indices,
                std::vector<std::pair<float, float>> &centers);

  void writeFile();

  void dance(float freq, float time);
  void stopDance();
  void moveHead(float angularSpeed=180.0);
  void stopHead();
  void equalizeTags();
  
private:
  // Data container that will be written to a csv file
  std::map<int, std::vector<std::vector<double> > > _container;
  std::map<int, std::vector<double> > _mapOfTagPositions;

  Leph::HumanoidFixedPressureModel _pastReadModel;
  Leph::HumanoidModel *_model;
  int _nbFramesInLog;
  std::string _fileName;
  int _nbFramesRead;
  bool _disabled;
  float _bpm;
  bool _dance;
  //float _elbowOffset;
  float _headSpeed;
  bool _handTuned;
  float _pan;
  float _tilt;
  
  float _hipPitchAmplitude;
  float _losangeAmplitude;
  
  float _trunkXAmplitude;
  float _trunkYAmplitude;
  float _trunkZAmplitude;
  
  float _t;
  int _maxFramesPerTag;
  
  
};
