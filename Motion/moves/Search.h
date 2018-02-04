#pragma once

#include <rhoban_utils/angle.h>
#include "Move.h"
#include "STM.h"

class Walk;
class Placer;
class Search : public STM
{
    public:
        Search(Walk *walk, Placer *placer);
        std::string getName();

        void onStart();
        void onStop();
        void step(float elapsed);

        float t;
        float getAzimuth();
        float getAzimuthError();
        float targetAzimuth;

        bool goLeft, goForward, goRight;

        int goDir;
        int nbRotate;
        
    protected:
        Walk *walk;
        Placer *placer;

        float rotateAfter, rotateAngle;
        float beginStep;

        float beginX, beginY, beginAzimuth;
        float P1X, P1Y, P1Azimuth;
        float P2X, P2Y, P2Azimuth;
        float lastBallY;
        int patrolTarget;

        void enterState(std::string state);
        void exitState(std::string state);
};
