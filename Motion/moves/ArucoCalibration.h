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
                std::vector<std::pair<float, float>> &centers,
                std::vector<std::pair<float, float>> &uncorrected_centers);

  void writeFile();

  void dance(float time);
  void stopDance();
  void moveHead();
  void stopHead();
  
private:
  // Data container that will be written to a csv file
  std::map<int, std::vector<std::vector<double> > > _container;
  std::map<int, std::vector<double> > _mapOfTagPositions;

  Leph::HumanoidFixedPressureModel _pastReadModel;
  Leph::HumanoidModel *_model;
  std::string _fileName;
  float _bpm;
  float _elbowOffset;
  float _shoulderRollOffset;
  float _headSpeed;
  
  float _hipPitchAmplitude;
  float _losangeAmplitude;
  
  float _trunkXAmplitude;
  float _trunkYAmplitude;
  float _trunkZAmplitude;
  
  /// Time since start of aruco
  float _t;
  
  /// ArucoCalibration starts smoothly over a given 
  float _smoothingTime;

  /// The duration of a log
  float _logDuration;
};
