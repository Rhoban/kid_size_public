#pragma once

#include <string>
#include <map>
#include <RhAL.hpp>
#include "services/Service.h"
#include <rhoban_utils/history/history.h>
#include <Model/HumanoidFixedPressureModel.hpp>
#include <Model/HumanoidFixedModel.hpp>

class Move;

enum FallDirection : int
{
  None = 0, Forward = 1, Backward = 2, Side = 3
};

/// TODO: threshold as variables
/// OK      : means no fall has been detected
/// Falling : means the robot is currently falling (error above 45 deg)
/// Fallen  : means the robot is almost (error above 75 deg)
enum FallStatus : int
{
  Ok = 0, Falling = 1, Fallen = 2
};

class DecisionService : public Service
{
    public:
        DecisionService();
 
        /**
         * Is the ball quality good enough?
         */
        bool isBallQualityGood;

        /// Is the ball currently moving according to the robot
        bool isBallMoving;

        /// Is one of the robot of the team currently kicking
        bool isMateKicking;

        /**
         * Is the field quality good enough?
         */
        bool isFieldQualityGood;

        /**
         * Is the robot fallen?
         * TODO: remove and use only fallStatus/fallDirection?
         */
        bool isFallen;

        /**
         * Should I let play?
         */
        bool shouldLetPlay;
        
        /**
         * Should I let play because of team?
         */
        bool shouldLetPlayTeam;

        /**
         * With what radius should I let play?
         */
        double letPlayRadius;
        
        /**
         * Should we go for the shared ball?
         */
        bool ballIsShared;

        /**
         * Am I the nearest robot to the ball ?
         */
        bool iAmTheNearest;
        
        /**
         * Position of the shared ball
         */
        float shareX, shareY;

        /**
         * Target of the kick of the ball handler
         */
        float ballTargetX, ballTargetY;

        /**
         * Is the robot handled?
         */
        bool handled;

        /**
         * Should the kick be frozen?
         */
        bool freezeKick;

        /**
         * Should we fake team decisons?
         */
        bool fakeTeamDecisions;

        /**
         * What is the side of the last seen ball?
         */
        bool lastSeenBallRight;

        /// Filtered direction of the fall
        FallDirection fallDirection;
        /// Filtered status of the fall
        FallStatus fallStatus;
    
    protected:
        RhIO::Bind bind;

        // Thresholds for ball quality
        float ballQThreshold, ballQDisableThreshold;

        /// Minimal ball speed to consider that ball speed is moving
        float movingBallMinSpeed;

        /// Time where a robot is forced to track the ball after a kick
        float postKickTrackingTime;

        // Thresholds for field quality
        float fieldQThreshold, fieldQDisableThreshold;
        
        // Ticking
        bool tick(double elapsed) override;

        // Low pressure threshold
        float lowPressureThreshold;
        
        /**
         * Should we let play our team
         */
        bool shouldLetTeamPlay(float ballDistance);

        /**
         * Should we listen to the other players?
         */
        bool cooperation;

        /**
         * Ball sharing
         */
        bool enableShare;
        int shareId;
        float shareFieldQ, shareBallQ;
        float shareSmooth;
        float handledT = 0;

        /**
         * Who is the goal?
         */
        int goalId;

        /**
         * Self attack detection
         */
        float selfAttackingT;

        /**
         * Counting time from free kick
         */
        double freeKickT;

        /// When has ball been flagged as moving for the last time
        rhoban_utils::TimeStamp lastBallMoving;
};

