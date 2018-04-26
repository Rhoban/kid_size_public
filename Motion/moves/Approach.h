#pragma once

#include "moves/ApproachMove.h"
#include <rhoban_utils/control/control.h>

class Walk;
class Head;
class Approach : public ApproachMove
{
    public:
        Approach(Walk *walk, Head *head);
        std::string getName();

        void onStart();
        void onStop();
        void step(float elapsed);
        
        // Clearing the ball
        bool clearing = false;

        void setPenalty(bool enable);
        
        // Time since last near ball
        float timeSinceNear;

    protected:
        // Head
        Head *head;

        float t;
        // Treshold to go for lateral shoot
        float lateralKickTreshold;
        bool enableLateralKick;
        float placementMargin;
        float targetPX, targetPY, theta;

        // Gain of the aligner
        float farAlignerP, rotateAlignerP, nearAlignerP;
        float rotateLateralP, nearLateralP;
        // Aligning with the ball
        rhoban_utils::Control aligner, stepper, lateraler;
        // Radius to rotate around the ball
        float radius;
        // Shoot time interval
        float shootInterval;
        float elapsedLastShoot;

        // Goal quality
        float goalQ;
        // Azimuth
        float ballAzimuth, goalAzimuth, goalLeftAzimuth, goalRightAzimuth;
        float ballX, ballY;
        int avoidDirection;
        // Distances
        float ballDistance;

        // Distance for centering
        float centeringCorner;

        // Number of shoots made
        int nbShoot;

        // Modes
        bool idiot;
        bool penalty;
        int penaltySide;

        void enterState(std::string state);
        void exitState(std::string state);
};
