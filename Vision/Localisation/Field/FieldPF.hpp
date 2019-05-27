#pragma once

#include "FieldPosition.hpp"
#include "FieldDistribution.hpp"

#include "rhoban_unsorted/particle_filter/particle_filter.h"

#include "RhIO.hpp"

#include <utility>
#include <vector>
#include <map>
#include <mutex>

namespace Vision
{
namespace Localisation
{
class FieldPF : public rhoban_unsorted::ParticleFilter<FieldPosition>
{
public:
  enum ResetType
  {
    None,
    Uniform,
    BordersRight,
    BordersLeft,
    Borders,
    Fall,
    Custom
  };

private:
  RhIO::IONode* rhioNode;

  ResetType resetType;

  // Points which should be tracked
  std::vector<int> errorTols;  // Tolerance to angle error to average

  virtual void updateInternalValues() override;

protected:
  virtual void updateRepresentativeQuality();
  virtual void updateRepresentativeParticle();
  cv::Mat positionsFromParticles();
  std::vector<rhoban_utils::Angle> anglesFromParticles();

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

  void step(rhoban_unsorted::Controller<FieldPosition>& ctrl,
            const std::vector<rhoban_unsorted::Observation<FieldPosition>*>& observations, double elapsedTime) override;

  void draw(cv::Mat& img) const;

  /// Return robot position in field [m]
  cv::Point2d getRobotPosition();
  cv::Point2d getCenterPositionInSelf();
  rhoban_utils::Angle getOrientation();
  double getQuality();

  bool isResetPending() const;

  static std::string getName(ResetType t);

  void initializeAtUniformRandom(unsigned int particlesNb);

  FieldDistribution fieldDistribution;

  double resamplingRatio;

  // Tolerance to error
  double tolDist;       // [m]
  double tolDiffAngle;  // [deg]

  double fallNoise;       // [m]
  double fallNoiseTheta;  // [deg]

  double borderNoise;       // [m]
  double borderNoiseTheta;  // [deg]
  double borderExtraDist;   // [m]

  double customX;           // [m]
  double customY;           // [m]
  double customTheta;       // [deg]
  double customNoise;       // [m]
  double customThetaNoise;  // [deg]

  void publishToRhIO();
  void importFromRhIO();

  /// Ensure thread safety access to 'resetType'
  std::mutex resetMutex;

  static std::map<ResetType, std::string> resetNames;
};
}  // namespace Localisation
}  // namespace Vision
