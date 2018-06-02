#pragma once

#include "STM.h"
#include <string>

/// Plays autonomously without listening to the referee
/// Just use automated standup and stop playing when handled
class StandUp;
class Walk;
class AutonomousPlaying : public STM
{
    public:
        AutonomousPlaying(Walk *walk, StandUp *standup);
        std::string getName();
        void onStart();
        void onStop();
        void step(float elapsed);
        bool isFallen();
        bool isHandled();

    protected:
        Walk *walk;
        StandUp *standup;

        float t;
        int standup_try;
        double timeSinceVisionInactive;

        void applyGameState();

        void enterState(std::string state);
        void exitState(std::string state);
};
