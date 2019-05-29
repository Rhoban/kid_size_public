#pragma once

#include <string>
#include <map>
#include <rhoban_utils/sockets/udp_broadcast.h>
#include <rhoban_team_play/team_play.h>
#include <hl_communication/udp_message_manager.h>
#include <hl_communication/wrapper.pb.h>
#include <RhIO.hpp>
#include "services/Service.h"

/**
 * TeamPlay
 *
 * UDP broadcast playing and robot state
 * to others players and monitoring
 */
class TeamPlayService : public Service
{
public:
  /**
   * Initialization
   */
  TeamPlayService();

  /**
   * Destructor
   */
  virtual ~TeamPlayService();

  /**
   * Return the robot teamplay id
   */
  int myId();

  /**
   * Access the teamplay info from current robot, field referential is in team point of view
   */
  const hl_communication::RobotMsg& selfInfo() const;

  /**
   * Access to the latest message from each player, field referential is in team point of view
   */
  const std::map<int, hl_communication::RobotMsg>& allInfo() const;
  std::map<int, hl_communication::RobotMsg> allInfoSafe();

  /**
   * Read/Write network and update outputs
   */
  bool tick(double elapsed);

  /**
   * Is team play enabled ?
   */
  bool isEnabled();

  /**
   * TeamPlay radius
   */
  double teamRadius;

  /**
   * Minimal distance to ball at kick off
   */
  double kickOffClearanceDist;

  /**
   * Minimal distance to ball for game interruptions (free-kick, corners, etc...)
   */
  double gameInterruptionClearanceDist;

private:
  /**
   * RhIO Binding
   */
  RhIO::Bind* _bind;

  /**
   * Protobuf message manager
   */
  std::unique_ptr<hl_communication::UDPMessageManager> message_manager;

  /**
   * Current and other robots infos
   */
  hl_communication::RobotMsg _selfInfo;
  std::map<int, hl_communication::RobotMsg> _allInfo;

  /**
   * Send and check
   * delay in seconds
   */
  double _t;

  /**
   * Enable or disable the teamplay
   */
  bool _isEnabled;

  /**
   * Broadcast period in seconds
   */
  double _broadcastPeriod;

  /**
   * Maximal number of obstacles in a teamplay message
   */
  int _maxObstacles;

  /**
   * Do message sending if needed
   */
  void messageSend();

  /**
   * Parse given message and update teamplay state
   */
  void processInfo(const hl_communication::RobotMsg& info);

  /**
   * Is the placing aggressive or defensive?
   */
  double aggressivity;

  /**
   * For hl_communication, the direction of x-axis for field referential is
   * based on the position of the team area. If the team attacks to the left of
   * the team area, then the field is inverted
   */
  bool _isFieldInverted;

  /**
   * Last team id used to send message. Is used to automatically change port when team number changed.
   */
  int last_team_id;

  /**
   * RhIO team command
   */
  std::string cmdTeam();

  /**
   * All info thread safety
   */
  std::mutex mutex;

  void updateIdentifier(hl_communication::RobotMsg* msg);
  void updatePerception(hl_communication::RobotMsg* msg);
  void updateIntention(hl_communication::RobotMsg* msg);
  void updateTeamPlay(hl_communication::RobotMsg* msg);
  void updateMiscExtra(hl_communication::RobotMsg* msg);
};
