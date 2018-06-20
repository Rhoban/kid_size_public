#include "ClearingKickController.h"
#include <services/LocalisationService.h>

ClearingKickController::ClearingKickController()
    : KickController()
{
    Move::initializeBinding();
    bind->bindNew("lateral", isInLateral, RhIO::Bind::PushOnly)
      ->defaultValue(false);

}

std::string ClearingKickController::getName()
{
    return "clearing_kick_controler";
}

void ClearingKickController::step(float elapsed)
{
  bind->pull();
  auto loc = getServices()->localisation;
  auto ball = loc->getBallPosField();

  if (isInLateral){
    if (fabs(ball.y)<1.8){
      isInLateral=false;
    } 
  } else { // not in lateral
    if (fabs(ball.y)>2){
      isInLateral=true;
    } 
  }
  if (isInLateral){
      allowed_kicks = {"lateral"};
      kick_dir = 0;
      tolerance = 50;    
  } else {
      allowed_kicks = {"classic", "lateral"};
      kick_dir = 0;
      tolerance = 80;    
  }
  bind->push();
}
