#include <stdlib.h>
#include <math.h>
#include <services/RefereeService.h>
#include <services/DecisionService.h>
#include <services/LocalisationService.h>
#include <services/TeamPlayService.h>
#include "Placer.h"
#include "Search.h"
#include "Walk.h"

// Waiting for the referee to start us
#define STATE_WAIT      "wait"
// We lost the ball, rotate around ourself to see it again
#define STATE_ROTATE    "rotate"
// We still don't go the ball, let's patrol
#define STATE_PATROL    "patrol"
// Begin target
#define STATE_BEGIN     "begin"
// Stopping
#define STATE_STOPPING  "stopping"
// Going to a shared ball position
#define STATE_SHARED    "shared"

using namespace rhoban_utils;

Search::Search(Walk *walk, Placer *placer)
    : walk(walk), placer(placer)
{
    initializeBinding();
    // State
    bind->bindNew("state", STM::state, RhIO::Bind::PushOnly)
        ->comment("Search STM state");
    bind->bindNew("beginStep", beginStep, RhIO::Bind::PullOnly)
        ->comment("Stepping used when the game is begining")
        ->defaultValue(30)->persisted(true);
    bind->bindNew("rotateAngle", rotateAngle, RhIO::Bind::PullOnly)
        ->comment("The angle that will be send to rotate when searching the ball")
        ->defaultValue(25)->persisted(true);
    bind->bindNew("rotateAfter", rotateAfter, RhIO::Bind::PullOnly)
        ->comment("The time before the robot begins its rotation when searching the ball")
        ->defaultValue(2.5)->persisted(true);

    bind->bindNew("beginX", beginX, RhIO::Bind::PullOnly)
        ->comment("Begin X")->defaultValue(0)->persisted(true);
    bind->bindNew("beginY", beginY, RhIO::Bind::PullOnly)
        ->comment("Begin Y")->defaultValue(0)->persisted(true);
    bind->bindNew("beginAzimuth", beginAzimuth, RhIO::Bind::PullOnly)
        ->comment("Begin Azimuth")->defaultValue(0)->persisted(true);

    bind->bindNew("P1X", P1X, RhIO::Bind::PullOnly)
        ->comment("Patrol checkpoint 1 X")->defaultValue(300)->persisted(true);
    bind->bindNew("P1Y", P1Y, RhIO::Bind::PullOnly)
        ->comment("Patrol checkpoint 1 Y")->defaultValue(0)->persisted(true);
    bind->bindNew("P1Azimuth", P1Azimuth, RhIO::Bind::PullOnly)
        ->comment("Patrol checkpoint 1 Azimuth")->defaultValue(0)->persisted(true);

    bind->bindNew("P2X", P2X, RhIO::Bind::PullOnly)
        ->comment("Patrol checkpoint 2 X")->defaultValue(-300)->persisted(true);
    bind->bindNew("P2Y", P2Y, RhIO::Bind::PullOnly)
        ->comment("Patrol checkpoint 2 Y")->defaultValue(0)->persisted(true);
    bind->bindNew("P2Azimuth", P2Azimuth, RhIO::Bind::PullOnly)
        ->comment("Patrol checkpoint 2 Azimuth")->defaultValue(180)->persisted(true);

    bind->bindNew("patrolTarget", patrolTarget, RhIO::Bind::PushOnly);

    bind->bindNew("t", t, RhIO::Bind::PushOnly);
    bind->bindNew("targetAzimuth", targetAzimuth, RhIO::Bind::PushOnly);
   
    bind->bindNew("goLeft", goLeft)
        ->defaultValue(false);
    bind->bindNew("goRight", goLeft)
        ->defaultValue(false);
    bind->bindNew("goForward", goLeft)
        ->defaultValue(false);

    // Note: the patrol target is keeped even if the placer is stopped and run
    // again
    patrolTarget = 0;
}

std::string Search::getName()
{
    return "search";
}

void Search::onStop()
{
    setState(STATE_STOPPING);
}

void Search::onStart()
{
    goLeft = false;
    goRight = false;
    goForward = false;
    nbRotate = 0;
    t = 0.0;
    setState(STATE_WAIT);
}

float Search::getAzimuth()
{
    return rad2deg(getGyroYaw());
}

void Search::step(float elapsed)
{
    auto decision = getServices()->decision;
    auto referee = getServices()->referee;
    bind->pull();
    t += elapsed;

    if (state == STATE_WAIT) {
        walk->control(false);

        if (referee->isBegining()) {
            setState(STATE_BEGIN);
        } else {
            if (t > rotateAfter) {
                setState(STATE_ROTATE);

                if (nbRotate == 2) {
                    setState(STATE_PATROL);
                } 
            }
        }
    }
    if (state == STATE_ROTATE) {
        float err = getAzimuthError();
        float turn = err;
        if (turn < -rotateAngle) turn = -rotateAngle;
        if (turn > rotateAngle) turn = rotateAngle;
        walk->control(true, 0, 0, turn);

        if (fabs(err) < 5.0) {
            nbRotate++;
            setState(STATE_WAIT);
        }
    }
    if (state == STATE_BEGIN) {
        if (referee->isBegining()) {
            placer->goTo(beginX/100, beginY/100, beginAzimuth);
        } else {
            setState(STATE_WAIT);
        }
    }
    if (state == STATE_SHARED) {
        placer->goTo(decision->shareX, decision->shareY, 0);
        if (placer->arrived) {
            setState(STATE_PATROL);
        }
    }
    if (state == STATE_PATROL) {
        if (placer->arrived && t > 3) {
            patrolTarget++;
            if (patrolTarget > 1) {
                patrolTarget = 0;
            }
            nbRotate = 0;
            setState(STATE_WAIT);
        }
    
        if (patrolTarget == 0) {
            placer->goTo(P1X/100, P1Y/100, P1Azimuth);
        } else {
            placer->goTo(P2X/100, P2Y/100, P2Azimuth);
        }
    }

    if (state == STATE_PATROL || state == STATE_BEGIN) {
        if (decision->ballIsShared) {
            setState(STATE_SHARED);
        }
    }
        
    bind->push();
}

void Search::enterState(std::string state)
{
    t = 0.0;

    if (state == STATE_PATROL || state == STATE_BEGIN || state == STATE_SHARED) {
        startMove("placer", 0.0);
    }
    if (state == STATE_ROTATE) {
        auto decision = getServices()->decision;
        double sign = decision->lastSeenBallRight ? 1 : -1;

        Angle result = Angle(getAzimuth()) + sign*Angle(120);
        targetAzimuth = result.getSignedValue();
        // XXX: This was disabled
        // loc->resetBallFilter();
    }
}

float Search::getAzimuthError()
{
    Angle result = Angle(targetAzimuth)-Angle(getAzimuth());
    return result.getSignedValue();
}

void Search::exitState(std::string state)
{
    if (state == STATE_PATROL || state == STATE_BEGIN || state == STATE_SHARED) {
        stopMove("placer", 0.0);
        walk->control(false);
    }
}
