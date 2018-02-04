#include "ClearingKickController.h"

ClearingKickController::ClearingKickController()
    : KickController()
{
    Move::initializeBinding();
}

std::string ClearingKickController::getName()
{
    return "clearing_kick_controler";
}

void ClearingKickController::step(float elapsed)
{
    allowed_kicks = {"classic", "lateral"};
    kick_dir = 0;
    tolerance = 80;
}
