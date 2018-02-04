#pragma once

#include "Move.h"
#include <rhoban_utils/spline/function.h>

#include <iostream>
#include <string>

class Replayer : public Move
{
    public:
        Replayer();
        ~Replayer();
        std::string getName();

        void onStart();
        void step(float elapsed);

        bool over;

    protected:
        float time;
        float speed;
        std::map<std::string, rhoban_utils::Function> splines;
        bool enable;
};
