#pragma once

#include <rhoban_geometry/point.h>
#include <rhoban_utils/serialization/json_serializable.h>
#include <rhoban_team_play/team_play.h>
#include <rhoban_utils/sockets/udp_broadcast.h>
#include <strategy/PlacementOptimizer.h>
#include <rhoban_utils/timing/time_stamp.h>
#include <thread>
#include "Service.h"

class ApproachMove;
class KickController;

class CaptainService : public Service
{
public:
  /**
   * Captain configuration
   */
  struct Config : public rhoban_utils::JsonSerializable
  {
    struct BasePosition
    {
      rhoban_geometry::Point targetPosition;
      bool mandatory;
      bool kickOff;
      float targetOrientation;
    };

    std::string getClassName() const;
    void fromJson(const Json::Value& json_value, const std::string& dir_name);
    Json::Value toJson() const;

    std::vector<BasePosition> basePositions;
  };

  CaptainService();
  virtual ~CaptainService();

  bool tick(double elapsed) override;

  /**
   * Returns the captain ID
   */
  int findCaptainId();

  /**
   * Am I the captain ?
   */
  bool amICaptain();

  /**
   * Getting last captain message, either received or generated
   * If no message has been received or built, returns an empty message
   */
  hl_communication::Captain getStatus();

  /**
   * Getting self order from captain message
   * If no orders are found, return an empty StrategyOrder
   */
  hl_communication::StrategyOrder getMyOrder();

  /**
   * Return false if there is no common_ball, otherwise return true and set 'dst' to the common ball in field
   * referential
   */
  bool importCommonBall(rhoban_geometry::Point* dst);

  /**
   * Update the captain status based on the provided message
   */
  void setStatus(const hl_communication::Captain& new_status);

protected:
  // RhIO binding
  RhIO::Bind bind;
  int captainId;
  bool IAmCaptain;
  bool share;

  // Captain configuration
  Config config;

  // Captain asynchronous thread
  volatile bool running;
  std::unique_ptr<std::thread> captainThread;
  std::mutex mutex;

  /**
   * This is a local copy of team play infos made by compute (threaded)
   */
  std::map<int, hl_communication::RobotMsg> teamPlayAllInfo;

  // Either the info computed or the one grabbed from the captain
  hl_communication::Captain status;

  /**
   * Storing last ball candidate to see if there
   */
  hl_communication::CommonBall last_common_ball;

  // Available robots
  std::map<int, hl_communication::RobotMsg> robots;
  std::vector<int> robotIds;

  /**
   * Frequency of the captain strategy update
   */
  double frequency;

  // Geometric parameters
  double passPlacingRatio, passPlacingOffset;
  double perpendicularBallDistance;
  double placingBallDistance;
  double avoidRadius;
  double minAggressivity, maxAggressivity;
  double aggressivity;

  /// Tolerance for merging parameters
  double commonBallTol;

  /// Minimal distance to a mate required for accepting an obstacle
  double oppToMateMinDist;

  /// Maximal distance for merging obstacles
  double oppMergeTol;

  /// Time during which a kick is considered as recent
  double kickMemoryDuration;

  /// Robot currently handling the ball
  int handler;

  /// The penalty received for changing the ball handler in teamplay
  double handlerChangeCost;

  /// The penalty for robots who are not seeing the ball
  double noViewCost;

  /// The penalty for robots who see a ball against consensus
  double wrongBallCost;

  /// Has a mate kicked recently
  bool recentlyKicked;

  /// Time since placing phase
  rhoban_utils::TimeStamp lastPlacingPhase;

  /// 0: disabled, 1: only when kick off, 2: always
  int openingStrategyMode;

  /// How much time in the begining of the game is the opening strategy ? [s]
  double openingStrategyTime;

  /// What orientation for the opening kick should be used ? [deg]
  double openingStrategyOrientation;

  // This is copied from the referee to avoid accessing it since we are in another thread
  bool myTeamKickOff;

  /**
   * Number of robots voting for the common ball
   */
  int commonBallStrength;
  /**
   * Position of the common ball according to the team in field referential
   */
  rhoban_geometry::Point commonBall;

  std::vector<PlacementOptimizer::Target> getTargetPositions(rhoban_geometry::Point ball,
                                                             rhoban_geometry::Point ballTarget);

  /**
   * Uses information from teamplay to reach a consensus on ball position
   * Do not clear potentially existing ball, this has to be done before
   */
  void updateCommonBall();

  /**
   * Use the obstacle information to extract a consensus on opponents
   * Removes the obstacles near allies and merge some of the other obstacles
   * Fill the *status* field
   */
  void updateCommonOpponents();

  /**
   * Do the update for the base positions (Ready state)
   * mutex has to be locked before calling the function
   */
  void computeBasePositions();

  /**
   * Do the update for the game position
   * mutex has to be locked before calling the function
   */
  void computePlayingPositions();

  /**
   * Computation of the captain orders
   * mutex has to be locked before calling the function
   */
  void compute();

  /**
   * Main loop of the background thread updating the robot positions
   */
  void execThread();

  /**
   * Updates the captain info from a placement optimizer solution
   * Does not affect players which are not in the solution
   */
  void setSolution(PlacementOptimizer::Solution solution);
};
