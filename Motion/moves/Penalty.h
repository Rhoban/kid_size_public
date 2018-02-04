#pragma once

#include "Move.h"

class PenaltyKickController;
class Penalty : public Move
{
    public:
        Penalty(PenaltyKickController *controler);
        std::string getName();

        void onStart();
        void step(float elapsed);

    protected:
        PenaltyKickController *controler;
        float startX;
        float T;
        bool forceStart;
        bool standingUp;
        bool watching;
        bool started;
};
