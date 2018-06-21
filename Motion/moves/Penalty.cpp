#include <math.h>
#include "Penalty.h"
#include <services/LocalisationService.h>
#include <services/DecisionService.h>
#include <services/RefereeService.h>
#include "PenaltyKickController.h"
#include "StandUp.h"
#include "Head.h"
#include "rhoban_utils/logging/logger.h"

static rhoban_utils::Logger logger("Penalty");
        
Penalty::Penalty(PenaltyKickController *controler)
    : controler(controler)
{
    initializeBinding();

    startX = 1.35;

    bind->bindNew("forceStart", forceStart, RhIO::Bind::PullOnly)
        ->comment("Force the start")->defaultValue(false);
}

std::string Penalty::getName()
{
    return "penalty";
}

void Penalty::onStart()
{
    stopMove("penalty_kick_controler");
    stopMove("approach_potential");
    startMove("head");
    startMove("walk");
    started = false;
    Head *head = dynamic_cast<Head*>(getMoves()->getMove("head"));
    head->setDisabled(false);
}

void Penalty::step(float elapsed)
{
    bind->pull();

    auto decision = getServices()->decision;
    auto loc = getServices()->localisation;
    auto referee = getServices()->referee;
    T += elapsed;

    if (!started && ((referee->isPlaying() && referee->myTeamKickOff()) || forceStart)) {
        auto loc = getServices()->localisation;
        logger.log("Let's go to the penalty attacker!");
        loc->penaltyReset(startX);
        started = true;
        T = 0;
        watching = true;
        stopMove("head");
        loc->resetRobotFilter();
    }
    if (started) {
        if (standingUp) {
            StandUp *standup = dynamic_cast<StandUp*>(getMoves()->getMove("standup"));
            if (standup->over) {
                stopMove("standup");
                startMove("walk");
                standingUp = false;
            } 
        } else {
            if (decision->isFallen) {
                standingUp = true;
                stopMove("walk");
                startMove("standup");
            }
            
            if (watching) {
                if (T > 1) {
                    T = 0;
                    watching = false;
                    startMove("head");
                    startMove("penalty_kick_controler");

                    auto opponents = loc->getOpponentsField();
                    if (opponents.size() == 1) {
                        double y = 0;
                        for (auto &opponent : opponents) {
                            y += opponent.y;
                        }
                        y = y/opponents.size();
                        bool left = y < 0;
                        logger.log("Saw %d opponents, kicking at %s", opponents.size(),
                                left ? "left" : "right");
                        controler->setLeft(left);
                    } else {
                        logger.log("No opponent saw, kicking at random");
                    }
                }
            } else {
                if (decision->isBallQualityGood && !decision->isFallen) {
                    startMove("approach_potential", 0.0);
                } else {
                    stopMove("approach_potential", 0.0);
                }
            }
        }
    }
    if (started && !referee->isPlaying() && !forceStart) {
        startMove("head");
        logger.log("Stopping the penalty kick, waiting");
        startMove("walk");
        stopMove("approach_potential", 0.0);
        stopMove("penalty_kick_controler", 0.0);
        stopMove("standup");
        started = false;
        watching = false;
    }
}
