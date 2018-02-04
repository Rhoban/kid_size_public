#pragma once

#include "Move.h"

class IMUTest : public Move
{
    public:
        IMUTest();
        std::string getName();

        void onStart();
        void step(float elapsed);

    protected:
        float amplitude;
        float freq;
        float t;
};
