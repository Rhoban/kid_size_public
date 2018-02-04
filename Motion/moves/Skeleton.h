#pragma once

#include "Move.h"

class Skeleton : public Move
{
    public:
        std::string getName();

        void onStart();
        void step(float elapsed);
};
