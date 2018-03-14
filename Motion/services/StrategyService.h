#pragma once

#include "Service.h"
#include "kick_model/kick_model_collection.h"

#include <rhoban_utils/timing/time_stamp.h>

class ApproachMove;
class KickController;

class StrategyService : public Service
{
public:
  StrategyService();

  const std::string & getDefaultApproach();
  const std::string & getDefaultKickController();

  const std::string & getActiveApproachName();
  const std::string & getActiveKickControllerName();

  ApproachMove * getActiveApproach();
  const KickController * getActiveKickController() const;

  bool tick(double elapsed) override;

  /// Inform the strategy service that a kick has been performed, 
  void announceKick();

protected:
  RhIO::Bind bind;

private:
  /// Default approach used
  std::string default_approach;

  /// Approach currently used
  std::string active_approach;

  /// Default kick_controler used
  std::string default_kick_controler;

  /// Kick controler currently used
  std::string active_kick_controler;

  /// When kicking the ball: target for the ball [m]
  double kick_target_x, kick_target_y;

  /// The tolerance of allowable kick [deg]
  double kick_tolerance;

  /// Grass offset [deg]
  double grass_offset;

  /// Kick controler azimuth [deg]
  double kick_controler_dir;

  /// Collection of available kicks
  csa_mdp::KickModelCollection kmc;

  /// When was last kick performed
  rhoban_utils::TimeStamp lastKick;
};
