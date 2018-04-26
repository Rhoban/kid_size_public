#include <services/LocalisationService.h>
#include <rhoban_utils/logging/logger.h>
#include "robocup_referee/constants.h"
#include "PenaltyKickController.h"

using namespace rhoban_geometry;
using namespace robocup_referee;

static rhoban_utils::Logger logger("penaltyKickController");

PenaltyKickController::PenaltyKickController()
    : KickController()
{
    Move::initializeBinding();
}

std::string PenaltyKickController::getName()
{
    return "penalty_kick_controler";
}

void PenaltyKickController::setLeft(bool left_)
{
    left = left_;
    logger.log("Seting explicitely side to %s", left ? "left" : "right");
}

void PenaltyKickController::onStart()
{
    left = time(NULL)&1;
    logger.log("Starting penalty kick controller, randoming to %s", left ? "left" : "right");
}

void PenaltyKickController::step(float elapsed)
{
    auto loc = getServices()->localisation;
    auto ball = loc->getBallPosField();
    Point target;

    // We kick either at 1/5 of the goal on the left or the right (much noise on kick dir)
    if (left) {
        target = Point(Constants::field.fieldLength/2, Constants::field.goalWidth/5);
    } else {
        target = Point(Constants::field.fieldLength/2, -Constants::field.goalWidth/5);
    }

    // Kicking with a powerful kick only to the target, 15° tolerance
    allowed_kicks = {"classic"};
    if (ball.x < target.x) {
        kick_dir = (target-ball).getTheta().getSignedValue();
    } else {
        kick_dir = 0;
    }
    tolerance = 10;
}
