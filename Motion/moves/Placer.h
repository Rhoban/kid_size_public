#pragma once

#include "Move.h"
#include "rhoban_utils/control/control.h"
#include <rhoban_geometry/circle.h>

class Walk;
class Placer : public Move
{
    public:
        Placer(Walk *walk);
        std::string getName();

        void onStart();
        void onStop();
        void step(float elapsed);

        // Controlling the placer, going to x, y [cm] and azimuth [deg]
        // The circle are obstacles [cm] to avoid
        void goTo(float x, float y, float azimuth, 
                  std::vector<rhoban_geometry::Circle> obstacles=std::vector<rhoban_geometry::Circle>());
        void setLateralMode(bool l);
        void setDirectMode(bool );
        float getMaxMarginXY();
        //void setTemporaryMarginAzimuth(float l);
        //void restoreMarginAzimuth();

        bool arrived;
        
        // Temporary target
        float tmpX, tmpY;

    protected:
        Walk *walk;

        bool dontWalk;
        float hysteresis;

        // Target point on the field [cm]
        float targetX, errorX;
        float targetY, errorY;

        // Target azimuth and its error [deg]
        float targetAzimuth, errorAzimuth;

        // Servoing classes
        rhoban_utils::Control stepper, lateraler, turner;

        // Margins [cm] and [deg]
        float marginX, marginY, marginAzimuth ;//, tmpMarginAzimuth;

        //Direct mode variables
        bool lateralMode;
        float lmDriftXMax;
        float lmDriftXMaxHyst;
        bool directMode, rushAngleOk, rushDistOk;
        float rushHysteresisLow, rushHysteresisHigh;
        float rushRadiusLow, rushRadiusHigh;
        float capToTarget;

        // Obstacles to avoid [cm]
        std::vector<rhoban_geometry::Circle> obstacles;

        // Distance to the next point that should be considered on the path [cm]
        float pathTargetDist;

        // Avoid the opponents ?
        bool avoidOpponents;

        // Json that represent the path like [[x1, y1], [x2, y2] ...]
        std::string pathJson;
};
