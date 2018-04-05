#pragma once

#include "moves/STM.h"

#include "kick_model/kick_model_collection.h"

#include <rhoban_geometry/point.h>
#include <rhoban_utils/angle.h>

class Walk;
class KickController;

class ApproachMove : public STM
{
public:
  ApproachMove(Walk * walk);

  /// Should be called after Move::initializeBinding() in child classes
  /// (class_name is virtual only in ApproachMove constructor)
  void initBindings();

  /// Reading link to kickController
  virtual void onStart() override;
  virtual void onStop() override;

  /// Return the target of the kick
  virtual rhoban_utils::Angle getKickCap();

  /// Returns the kick tolerance, +/- [deg]
  virtual double getKickTolerance();

  virtual std::vector<std::string> getAllowedKicks();

  /// Is given kick among the list of kicks allowed?
  bool isKickAllowed(const std::string & name);

  /// Return the name of the currently expected kick
  const std::string & getExpectedKick() const;

  /// Request kick with the appropriate parameters to walk
  /// (kick used is 'expectedKick')
  virtual void requestKick();

  /// Return true if the KickController exists and is running
  bool useKickController() const;

protected:
  /// Increase or decrease kickScore depending if ball is considered as kickable
  /// Warning: expectedKick and kickRight have to be set properly before
  ///          calling this function
  void updateKickScore(double elapsed);

  /// @see updateKickScore
  /// ball_pos is provided in self referential
  void updateKickScore(double elapsed, const rhoban_geometry::Point & ball_pos);

  /// Gets the current kick controler
  const KickController * getKickController() const;

  /// Link to the walk move
  Walk * walk;

  /// Current expectation of the kick
  std::string expectedKick;

  /// Is next kick supposed to use right foot? (false -> left)
  bool kickRight;

  /// Kick score in [0,1]
  /// When score reaches one, robot is allowed to shoot
  double kick_score;

  /// Score increases by elapsed * kick_gain when ball is kickable
  double kick_gain;
  
  /// Score decreases by elapsed * no_kick_gain when ball is not kickable
  double no_kick_gain;

  /// The list of available kicks
  csa_mdp::KickModelCollection kmc;

  // Enable lateral kick
  bool enableLateral;
};
