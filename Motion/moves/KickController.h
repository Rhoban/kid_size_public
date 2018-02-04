#pragma once

#include "moves/Move.h"

#include "problems/kick_controler.h"//from csa_mdp_experiments

#include "rhoban_utils/angle.h"

class Walk;

class KickController : public Move
{
public:
  KickController();

  /// Should be called once Move::initializeBindings() has been called
  void initBindings();

  /// Return the wished direction for the next kick
  rhoban_utils::Angle getKickDir() const;

  /// Returns the accepted tolerance for the next kick +/- [deg] 
  double getTolerance() const;

  const std::vector<std::string> & getAllowedKicks() const;

protected:
  /// Which kicks are currently allowed by the kick_controler
  std::vector<std::string> allowed_kicks;

  /// Which direction should be used for the kick [deg]
  double kick_dir;

  /// Tolerance [deg]
  double tolerance;
};
