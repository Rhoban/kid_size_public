#include <math.h>
#include "Placer.h"
#include "Robocup.h"
#include <RhIO.hpp>
#include <services/RefereeService.h>
#include <services/LocalisationService.h>
#include <services/DecisionService.h>
#include <scheduler/MoveScheduler.h>
#include "StandUp.h"
#include "Head.h"
#include "Walk.h"

#include "rhoban_utils/logging/logger.h"

#define STATE_INITIAL   "initial"
#define STATE_WAITING   "waiting"
#define STATE_PLAYING   "playing"
#define STATE_STANDUP   "standing_up"
#define STATE_STOPPING  "stopping"
#define STATE_PLACING   "placing"
#define STATE_PENALIZED "penalized"
#define STATE_GIVE_UP   "give_up"

static rhoban_utils::Logger logger("RobocupSTM");

using namespace rhoban_geometry;

Robocup::Robocup(Walk *walk, StandUp *standup, Placer *placer)
    : walk(walk), standup(standup), placer(placer)
{
    initializeBinding();
    // State
    bind->bindNew("state", STM::state, RhIO::Bind::PushOnly)
        ->comment("State of the Robocup STM");
    bind->bindNew("goalKeeper", goalKeeper, RhIO::Bind::PullOnly)
        ->comment("Am I a goal keeper?")
        ->defaultValue(false);
    bind->bindNew("freeKicker", freeKicker, RhIO::Bind::PullOnly)
        ->comment("Am I the free kicker performer?")
        ->defaultValue(false);
    bind->bindNew("autoKickOff", autoKickOff, RhIO::Bind::PullOnly)
        ->comment("Are the kickOff performed autonomously?")
        ->defaultValue(true);
    // Initial locations for autoPlacing (Position at which the robot is dropped)
    bind->bindNew("autoStartX", autoStartX, RhIO::Bind::PullOnly)
        ->comment("Start of the robot during ready phase [m]")
        ->defaultValue(-1.5);
    bind->bindNew("autoStartY", autoStartY, RhIO::Bind::PullOnly)
        ->comment("Start of the robot during ready phase [m]")
        ->defaultValue(3);
    bind->bindNew("autoStartAzimuth", autoStartAzimuth, RhIO::Bind::PullOnly)
        ->comment("Initial orientation of the robot during ready phase [deg]")
        ->defaultValue(-90);
    // Target for autoPlacing
    bind->bindNew("autoTargetX", autoTargetX, RhIO::Bind::PullOnly)
        ->comment("Target of the robot during ready phase [m]")
        ->defaultValue(-1.5);
    bind->bindNew("autoTargetY", autoTargetY, RhIO::Bind::PullOnly)
        ->comment("Target of the robot during ready phase [m]")
        ->defaultValue(0);
    // Target location for the kicker
    bind->bindNew("freeKickX", freeKickX, RhIO::Bind::PullOnly)
        ->comment("Target of the robot performing freeKick during ready phase [m]")
        ->defaultValue(-0.5);
    bind->bindNew("freeKickY", freeKickY, RhIO::Bind::PullOnly)
        ->comment("Target of the robot performing freeKick during ready phase [m]")
        ->defaultValue(0);
    // Time since vision
    bind->bindNew("timeSinceVisionInactive", timeSinceVisionInactive, RhIO::Bind::PushOnly);
}

std::string Robocup::getName()
{
    return "robocup";
}

void Robocup::onStart()
{
    bind->pull();
    standup_try = 0;
    setState(STATE_WAITING);
    startMove("head", 0.5);
    lastRefereePlaying = false;
    rememberStart = false;
    wasHandled = true;
    walk->control(false);
}

void Robocup::onStop()
{
    stopMove("head");
    setState(STATE_STOPPING);
}

void Robocup::applyGameState()
{    
    auto &decision = getServices()->decision;
    auto referee = getServices()->referee;
    auto loc = getServices()->localisation;

    // If we are at the beginning of the game, jump to state_initial
    if (state != STATE_INITIAL && referee->isInitialPhase()) {
        setState(STATE_INITIAL);
        logger.log("Jumping to Initial State");
    }

    // If:
    // - mode is auto
    // - phase ready just started
    // Then, try to place autonomously
    if (autoKickOff &&
        state != STATE_PLACING && referee->isPlacingPhase() &&
        !wasHandled) {
        logger.log("Jumping to Placing Phase");
        // If we were previously in initial state, reset particle filters
        if (state == STATE_INITIAL) {
            double locationNoise = 30;
            double azimuthNoise = 10;
            loc->customFieldReset(autoStartX, autoStartY, locationNoise,
                                  autoStartAzimuth, azimuthNoise);
            logger.log("Initial reset at  x: %f y: %f with theta: %f",
                       autoStartX, autoStartY, autoStartAzimuth);
        }
        setState(STATE_PLACING);
    }

    // If mode is freeze, jump to waiting state
    if (state != STATE_WAITING && referee->isFreezePhase()) {
        logger.log("Jumping to Wait Phase");
        setState(STATE_WAITING);
    }

    // If we are allowed to play while we were not allowed previously
    bool weCanNowPlay = referee->isPlaying() && !referee->isFreezePhase() && !lastRefereePlaying;
    if (weCanNowPlay || rememberStart) {
        if (decision->handled) {
            // We are handled, waiting to be put on the floor to start playing to
            // avoid reseting filters on a bad start position, postponing the start
            rememberStart = true;
        } else {
            rememberStart = false;
            logger.log("Is now playing");

            // There is a free kick pending, just continuing to play normally
            if (referee->isFreeKick()) {
                logger.log("End of free kick freeze phase, going back to play");
            }
            // If player was penalized before
            else if (referee->wasPenalized) {
                if (wasHandled) {
                    // We was penalized and handled, resetting the filter to the borders of
                    // the field
                    logger.log("Playing from bordersReset (penalized)");
                    loc->bordersReset();
                } else {
                    // Maybe the referee misclicked us, if we was not handled we should not
                    // reset the filters to the border
                    logger.log("I am unpenalized but I was not handled, not resetting filters");
                }
            }
            // If autoKickOff and not goalie, do not reset and start playing
            else if (autoKickOff) {
                logger.log("Playing with autoKickOff");
                if (wasHandled) {
                    // We was handled, so we suppose that we was not placed correctly and
                    // resetting the filters on the game start
                    if (goalKeeper) {
                        logger.log("I was handled during the placing phase, reseting the filters in the goals");
                        loc->goalReset();
                    } else {
                        logger.log("I was handled during the placing phase, reseting the filters");
                        loc->gameStartReset();
                    }
                }
            }
            // FreeKicker case
            else if (freeKicker && referee->myTeamKickOff()) {
                logger.log("Starting manual freeKicker");
                loc->kickOffReset();
            }
            // GoalKeeper case (if no auto kick off)
            else if (goalKeeper) {
                logger.log("Starting manual GoalKeeper");
                loc->goalReset();
            }
            // Classic start
            else {
                logger.log("Starting classical game");
                loc->gameStartReset();
            }
            setState(STATE_PLAYING);
        }
    }

    // If we are allowed to play while we were not allowed previously
    if (!rememberStart && !referee->isFreezePhase() && referee->isPlaying() && state == STATE_WAITING) {
        logger.log("Entering playing");
        setState(STATE_PLAYING);
    }
    
    if (!referee->isPlaying() && state == STATE_PLAYING) {
        setState(STATE_WAITING);
    }
}

void Robocup::step(float elapsed)
{
    auto &decision = getServices()->decision;
    bind->pull();

    // We gave up, just die
    if (state == STATE_GIVE_UP) {
        if (standup->over) {
            stopMove("standup", 0.0);
            stopMove("head", 0.0);
            stopMove("walk", 0.0);
            getScheduler()->releaseServos();
        }
        return;
    }

    // Detect if the robot is being handled
    if (decision->handled) {
        wasHandled = true;
    }

    t += elapsed;
    auto &referee = getServices()->referee;
    bool isPlaying = referee->isPlaying();
    bool isPenalized = referee->isPenalized();

    /// Let standup finish if it started, otherwise go to penalized state if
    /// referee asks to
    if (state != STATE_STANDUP && state != STATE_PENALIZED && isPenalized) {
        logger.log("Robot is penalized, waiting information from the referee");
        setState(STATE_PENALIZED);
    }
    /// Go through state Waiting if we were penalized and we are not penalized anymore
    if (state == STATE_PENALIZED && !isPenalized) {
        logger.log("Robot is leaving penalized, going to waiting");
        setState(STATE_WAITING);
    }

    // Fall recovery start from waiting state
    if (state == STATE_WAITING && decision->isFallen) {
        logger.log("Robot has fallen, standing up");
        setState(STATE_STANDUP);
    }
    // Only apply gameState if robot is not standing up and not penalized
    if (state != STATE_STANDUP && state != STATE_PENALIZED) {
        applyGameState();
    }
    // If robot was standing up standup has finished, jump to Waiting state
    if (state == STATE_STANDUP && standup->over) {
        logger.log("Standup has finished, back to Waiting");
        setState(STATE_WAITING);
    }
    // When the robot falls, go through waiting state (TODO, check if its necessary)
    if (decision->isFallen && (state == STATE_PLAYING || state == STATE_PLACING)) {
        logger.log("Robot has fallen, going to wait");
        setState(STATE_WAITING);
    }
    // If the robot is being handled during the placing, go to waiting
    if (state == STATE_PLACING) {
        if (decision->handled) {
            setState(STATE_WAITING);
        }
    }

    // Spam stop orders to walk when waiting
    if (state == STATE_WAITING) {
        walk->control(false);
    }

    if (state == STATE_INITIAL || state == STATE_PLAYING) {
        wasHandled = false;
    }

    auto loc = getServices()->localisation;
    if (state == STATE_PLAYING) {
        // Vision is inactive
        if (!loc->isVisionActive()) {
            timeSinceVisionInactive += elapsed;
            if (timeSinceVisionInactive > 30) {
                setState(STATE_GIVE_UP);
                logger.log("I lost my camera for more than 30s, giving up!");
            }
        } else {
            timeSinceVisionInactive = 0;
        }
        /*
        // We are attacking ourself, stop this!
        if (decision->isSelfAttacking) {
            setState(STATE_GIVE_UP);
            logger.log("I am attacking my own team?! Giving up");
        }
        */
    } else {
        timeSinceVisionInactive = 0;
    }
    
    // Backuping old values
    lastRefereePlaying = isPlaying && !isPenalized;

    bind->push();
}

void Robocup::enterState(std::string state)
{ 
    auto referee = getServices()->referee;

    t = 0;

    Head * head = (Head*)getMoves()->getMove("head");
    // Not scanningm only if the robot is penalized
    if (state == STATE_PENALIZED) {
        head->setDisabled(true);
    } else {
        head->setDisabled(false);
    }
    
    if (state == STATE_STANDUP) {
        standup->setLayDown(false);
        stopMove("walk", 0.3);
        startMove("standup", 0.0);
        standup->trying = standup_try;
    } else {
        startMove("walk", 1.0);
        walk->control(false);
    }

    if (state == STATE_GIVE_UP) {
        standup->setLayDown(true);
        walk->control(false);
        stopMove("walk", 0.3);
        startMove("standup", 0.0);
    }

    if (state == STATE_PLAYING) {
        rememberStart = false;
        if (goalKeeper) {
            startMove("goal_keeper", 0.0);
        }
        else {
            startMove("playing", 0.0);
        }
    }

    if (state == STATE_PLACING) {
        startMove("placer");
        double targetX, targetY;
        std::vector<Circle> obstacles;
        // Special for freeKicker
        if (freeKicker && referee->myTeamKickOff() && !referee->isDroppedBall()) {
            targetX = freeKickX;
            targetY = freeKickY;
        }
        // Classic Target
        else {
            targetX = autoTargetX;
            targetY = autoTargetY;
            obstacles.push_back(Circle(Point(0, 0), 1.2));
        }
        logger.log("Placing to x: %f y: %f", targetX, targetY);
        placer->goTo(targetX, targetY, 0, obstacles);
    }
}

void Robocup::exitState(std::string state)
{
    auto &decision = getServices()->decision;

    // After standing up:
    // - Apply a fallReset on filters
    // - Stop the standup move
    if (state == STATE_STANDUP) {
        auto loc = getServices()->localisation;
        loc->fallReset();
        stopMove("standup", 0.0);
        // Used to slow next standup if first one failed
        if (decision->isFallen) {
            standup_try++;
        } else {
            standup_try = 0;
        }
    }

    // Stop main move when leaving playing
    if (state == STATE_PLAYING) {
        if (goalKeeper) {
            stopMove("goal_keeper", 0.0);
        } else {
            stopMove("playing", 0.0);
        }
    }

    // Destroy the placer
    if (state == STATE_PLACING) {
        stopMove("placer", 0.0);
        walk->control(false);
    }

    // End the player move 
    if (state == STATE_PLAYING) {
        if (goalKeeper) {
            stopMove("goal_keeper", 0.0);
        }
        else {
            stopMove("playing", 0.0);
        }
    }
}
