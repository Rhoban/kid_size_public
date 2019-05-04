#pragma once

#include <vector>
#include "Service.h"
#include <robocup_referee/referee_client.h>

namespace RhIO
{
class Bind;
}

class RefereeService : public Service, public robocup_referee::RefereeClient
{
public:
  RefereeService();
  ~RefereeService();

  /**
   * Updating the playing flag
   */
  void checkPlaying();

  /**
   * Am I playing ?
   * (This is false when penalized)
   */
  bool isPlaying();

  /**
   * Is the game playing ?
   * (This is true even when penalized)
   */
  bool isGamePlaying();

  /// This phase happens only at the beginning of the game
  bool isInitialPhase();

  /// During this phase robots are allowed to move to their own field
  bool isPlacingPhase();

  /// During this phase robot should freeze
  bool isFreezePhase();

  /// End of game / end of half
  bool isFinishedPhase();

  /**
   * Is the robot playing since a short time.
   */
  bool hasStartedPlayingRecently();

  /**
   * Time since playing (reseted if penalized)
   */
  double getTimeSincePlaying();

  /**
   * True if opponent team has kickOff recently and rules forbid to touch the ball
   */
  bool isOpponentKickOffStart();

  /**
   * Am I penalized?
   */
  bool isPenalized();

  /**
   * Is the given player penalized?
   */
  bool isPenalized(int id);

  /**
   * Is our team taking the kickoff?
   */
  bool myTeamKickOff();

  /**
   * Are we in a dropped ball scenario ?
   */
  bool isDroppedBall();

  /**
   * Is there a game interruption right now?
   */
  bool isGameInterruption();

  /**
   * Are we in the phase after a game interruption?
   */
  bool isRecentGameInterruption();

  /**
   * Is the current or last game_interruption for our team?
   */
  bool myTeamGameInterruption();

  /**
   * Is the current or last game interruption a throw-in?
   */
  bool isThrowIn();

  /**
   * Secondary time
   */
  int getSecondaryTime();

  int gameTime();

  bool tick(double elapsed) override;
  double timeSincePlaying;
  double timeSinceGamePlaying;
  int remaining;

  int id, teamId;
  int alive;
  bool force;
  /**
   * Determines if the robot should play, based on game controller state. False if robot is penalized.
   * Can be forced through the 'force' flag.
   */
  bool playing;
  bool gamePlaying;

  bool wasPenalized;

  bool dumpGameState;

protected:
  RhIO::Bind* bind;
  float startPlayingDuration;

  /**
   * Textual referee state
   */
  std::string _state;

  std::string cmdPlaying();

  void setTextualState();

  /**
   * The time in seconds since a game interruption was finished.
   */
  float timeSinceGameInterruption;

  /**
   * If there has been no game interruption since the start, then the value is 0
   */
  int lastGameInterruptionType;
};
