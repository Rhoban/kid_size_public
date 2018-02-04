#include "KickController.h"

KickController::KickController() {}

void KickController::initBindings()
{
  bind->bindNew("kick_dir", kick_dir, RhIO::Bind::PushOnly)
    ->defaultValue(0);
  bind->bindNew("tolerance", tolerance, RhIO::Bind::PushOnly)
    ->defaultValue(0);
}


rhoban_utils::Angle KickController::getKickDir() const
{
  return rhoban_utils::Angle(kick_dir);
}

double KickController::getTolerance() const
{
  return tolerance;
}

const std::vector<std::string> & KickController::getAllowedKicks() const
{
  return allowed_kicks;
}
