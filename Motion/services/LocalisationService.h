#pragma once

#include <RhIO.hpp>
#include "Service.h"

#include <Eigen/Dense>
#include <rhoban_geometry/point.h>
#include <rhoban_utils/timing/time_stamp.h>
#include <Localisation/Field/FieldDistribution.hpp>

namespace Vision
{
class Robocup;
class LocalisationBinding;
namespace Localisation
{
class FieldDistribution;
}

}  // namespace Vision

/// Unless explicitely stated otherwise, units are SI
class LocalisationService : public Service
{
public:
  LocalisationService();

  // Ball
  rhoban_geometry::Point getBallPosSelf();
  rhoban_geometry::Point getBallPosWorld();
  rhoban_geometry::Point getBallPosField();
  rhoban_geometry::Point getBallSpeedSelf();
  rhoban_geometry::Point getPredictedBallSelf();
  /**
   * Returns the predicted ball position in self referential at time 't', using linear interpolation
   */
  rhoban_geometry::Point getPredictedBallSelf(rhoban_utils::TimeStamp t);
  Eigen::Vector3d ballPosWorld;
  float ballQ;
  rhoban_geometry::Point ballSpeed;  // In world referential
  rhoban_utils::TimeStamp ballTS;

  // Goal
  rhoban_geometry::Point getGoalPosField();
  rhoban_geometry::Point getOurGoalPosField();
  /**
   * Returns the position of the robot in field referential [m]
   */
  rhoban_geometry::Point getFieldPos();
  /**
   * Return the orientation of the robot in field referential [rad]
   */
  double getFieldOrientation();
  Eigen::Vector3d fieldCenterWorld;
  rhoban_utils::Angle getOurBallToGoalDirSelf();

  float fieldQ, fieldConsistency;
  bool consistencyEnabled;

  // Opponent
  std::vector<Eigen::Vector3d> getOpponentsSelf();
  std::vector<rhoban_geometry::Point> getOpponentsField();
  double opponentsRadius;
  bool opponentsAreFake;
  std::vector<Eigen::Vector3d> opponentsWorld;
  void updateOpponentsPos();
  void setOpponentsWorld(const std::vector<Eigen::Vector3d>& pos);

  /// Update the string representing shared_opponents
  void updateSharedOpponentsPos();

  /// Return a thread safe copy of the team mates positions by Id
  std::map<int, Eigen::Vector3d> getTeamMatesField();

  /// Return the list of opponents seen by other mates
  std::vector<Eigen::Vector2d> getSharedOpponents();

  /// robot_id is the robot providing the opponent Positions
  void updateSharedOpponents(int robot_id, const std::vector<Eigen::Vector2d>& opponentPos);

  /// Remove all obstacles provided by this robot
  void removeSharedOpponentProvider(int id);

  /// Occupation of a teamMate for obstacle avoidance [m]
  double teamMatesRadius;

  /// Stored position of the team mates currently playing
  /// values: (x[m], y[m], dir[rad])
  std::map<int, Eigen::Vector3d> teamMatesField;

  /// Key: id of robot providing the information
  /// Value: List of opponents currently seen by the robot
  std::map<int, std::vector<Eigen::Vector2d>> sharedOpponentsField;

  // Tell the localisation service we see the ball
  void setBallWorld(const Eigen::Vector3d& pos, float quality, const rhoban_geometry::Point& speed,
                    const rhoban_utils::TimeStamp& ts);
  void updateBallPos();
  void setNoBall();

  /**
   * Updates the location of the field in the world referential based on information in self referential
   * - center: position of center in self basis [m]
   * - orientation: orientation of the robot on the field [rad]
   */
  void setPosSelf(const Eigen::Vector3d& center_in_self, float orientation, float quality, float consistency,
                  bool consistencyEnabled = false, bool replayValue = false);

  /**
   *Updates all possibles position calculated with EM algorithm
   */
  void setCluster(std::vector<hl_communication::WeightedPose*> candidates);

  std::vector<hl_communication::WeightedPose*> getPositionInClusters();

  /**
   * Updates transforms between field and world basis and then updates variables monitored by RhIO
   */
  void updatePosSelf();

  /// x,y -> [m]
  void applyKick(float x, float y);

  // How many frames were captured by the camera?
  int getFrames();

  // Accessing vision status
  std::string getCameraStatus();
  double getLastVisionUpdate();
  bool isVisionActive() const;

  void resetBallFilter();
  void resetFieldFilter();
  void resetRobotFilter();

  void kickOffReset();
  void dropBallReset();
  /// x in [m]
  void penaltyReset(float x);
  void penaltyGoalReset();
  void goalReset();

  /// x,y, noise in meters, theta thetaNoise in deg
  void customFieldReset(double x, double y, double noise, double theta, double thetaNoise);
  void fallReset();
  void bordersReset();

  /**
   * Convert given position from world refential to field referential (not thread safe)
   */
  rhoban_geometry::Point worldToField(const Eigen::Vector3d& pos_in_world);
  Eigen::Vector3d fieldToWorld(const Eigen::Vector3d& pos_in_field);

  void setGoalKeeper(bool status = false);

  void setRobocup(Vision::Robocup* robocup);
  void setLocBinding(Vision::LocalisationBinding* locBinding);

  bool isReplay;

protected:
  // Mutex to access ball position
  std::mutex mutex;

  // For RhIO
  float ballPosX, ballPosY;
  float ballFieldX, ballFieldY;
  // Informations provided by the speed estimator
  float ballSpeedXInSelf, ballSpeedYInSelf;
  float ballPredictedX, ballPredictedY;

  // For RhIO
  float fieldPosX, fieldPosY;

  /**
   * The orientation of the robot in field referential [deg]
   */
  float fieldOrientation;
  /**
   * The field orienta
   */
  float fieldOrientationWorld;
  std::string opponents;
  std::string mates;
  std::string sharedOpponents;

  // Ticking
  bool tick(double elapsed) override;

  RhIO::Bind bind;
  // Used for warning the user on frequent kicks
  ::rhoban_utils::TimeStamp lastKick;

  Vision::Robocup* robocup;
  Vision::LocalisationBinding* locBinding;
  std::vector<hl_communication::WeightedPose*> posFromClusters;

  /**
   * Update the basis transforms between field and world based on the following internal information.
   * - fieldCenterWorld
   * - fieldOrientationWorld
   */
  void updateFieldWorldTransforms();

  /**
   * Update the basis transforms between self and world based on ModelService
   * - If fake mode is enabled, uses goalModel
   * - If fake mode is disabled, uses correctedModel
   */
  void updateSelfWorldTransforms();

  /**
   * Return the direction of the trunk in world referential
   */
  rhoban_utils::Angle getSelfOrientationInWorld();

public:
  // Conversions between self, field and world referentials
  Eigen::Affine3d field_from_world;
  Eigen::Affine3d world_from_field;
  Eigen::Affine3d self_from_world;
  Eigen::Affine3d world_from_self;

  /**
   * RhIO command to set the ball /goal position in fake mode
   */
  std::string cmdFakeBall(double x, double y);
  std::string cmdFakeOpponents(std::vector<std::string> args);
  std::string cmdFakeLoc(double fieldX, double fieldY, double orientation);
  std::string cmdResetPosition();
  std::string cmdMoveOnField(double x, double y, double yaw);

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
