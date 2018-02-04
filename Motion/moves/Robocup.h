#pragma once

#include "STM.h"
#include <string>

class Placer;
class StandUp;
class Walk;
class Robocup : public STM
{
    public:
        Robocup(Walk *walk, StandUp *standup, Placer *placer);
        std::string getName();
        bool standup_disabled;
        void onStart();
        void onStop();
        void step(float elapsed);
        bool isFallen();
        bool isHandled();

    protected:
        Walk *walk;
        StandUp *standup;
        Placer * placer;

        float t;
        bool wasHandled;
        bool lastRefereePlaying;
        bool rememberStart;
        bool goalKeeper;
        bool freeKicker;
        float freeKickerExtraX;
        bool autoKickOff;
        int standup_try;
        double timeSinceVisionInactive;

        // Robot position at end of initial phase for autoKickOff
        float autoStartX;
        float autoStartY;
        float autoStartAzimuth;
        // Target of the robot for autoKickOff when not being freeKicker
        float autoTargetX;
        float autoTargetY;
        // Target of the robot for autoKickOff when freeKicker
        float freeKickX;
        float freeKickY;

        void applyGameState();

        void enterState(std::string state);
        void exitState(std::string state);
};
