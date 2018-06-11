#pragma once

#include "FieldPosition.hpp"
#include "Field/Field.hpp"

#include "CameraState/CameraState.hpp"

#include "rhoban_unsorted/particle_filter/particle_filter.h"

#include "RhIO.hpp"
#include <utility>
#include <vector>
#include <map>
#include <mutex>

namespace Vision {
namespace Localisation {

class FieldPF : public rhoban_unsorted::ParticleFilter<FieldPosition> {
public:
  typedef std::pair<int, int> GoalKey;
  typedef std::tuple<int, int, int> QualityKey;
  enum ResetType { None, Uniform, BordersRight, BordersLeft, Borders, Fall, Custom };

private:
  RhIO::IONode *rhioNode;

  ResetType resetType;

  // Points which should be tracked
  std::vector<GoalKey> trackedGoals;
  std::vector<int> errorTols; // Tolerance to angle error to average

  // Directions and qualities of goals points of interest
  std::map<GoalKey, rhoban_utils::Angle> goalDirections;
  std::map<QualityKey, double> goalQualities;

  virtual void updateInternalValues() override;
  void updateToGoalQuality();

protected:
  virtual void updateRepresentativeQuality();
  virtual void updateRepresentativeParticle();

public:
  FieldPF();

  /// Overwrite current resetType if it has not been ticked yet
  void askForReset(ResetType t = ResetType::Uniform);

  /// If current reset type is of type t, then replace it by 'None'
  void cancelPendingReset(ResetType t);

  ResetType getPendingReset();

  // side: -1 for right, 1 for left, 0 for unknownside
  void resetOnLines(int side);
  void fallReset();
  void customReset();
  /// Apply the pending reset if there is one (and set next reset to None)
  void applyPendingReset();
  void applyReset(ResetType t);

  void step(rhoban_unsorted::Controller<FieldPosition> &ctrl,
            const std::vector<rhoban_unsorted::Observation<FieldPosition> *> &observations,
            double elapsedTime) override;

  void draw(cv::Mat &img, Utils::CameraState *cs) const;

  rhoban_utils::Angle getAngleToGoal(const GoalKey &gk) const;
  double angleToGoalQuality(const QualityKey &qk) const;

  rhoban_utils::Angle getAngleToGoal();
  double angleToGoalQuality();

  /// Return robot position in field [m]
  cv::Point2d getRobotPosition();
  cv::Point2d getCenterPosition();
  cv::Point2d getLeftGoalPosition();
  cv::Point2d getRightGoalPosition();
  cv::Point2d getCenterPositionInSelf();
  cv::Point2d getLeftGoalPositionInSelf();
  cv::Point2d getRightGoalPositionInSelf();
  rhoban_utils::Angle getOrientation();
  double getQuality();

  bool isResetPending() const;

  static std::string getName(ResetType t);

  void initializeAtUniformRandom(unsigned int particlesNb);

  double resamplingRatio;

  // Tolerance to error
  double tolDist;      // [m]
  double tolDiffAngle; // [deg]

  double fallNoise;// [m]
  double fallNoiseTheta;// [deg]

  double borderNoise;// [m]
  double borderNoiseTheta;// [deg]

  double customX;// [m]
  double customY;// [m]
  double customTheta;// [deg]
  double customNoise;// [m]
  double customThetaNoise;// [deg]

  void publishToRhIO();
  void importFromRhIO();

  /// Ensure thread safety access to 'resetType'
  std::mutex resetMutex;

  static std::map<ResetType, std::string> resetNames;
};
}
}
